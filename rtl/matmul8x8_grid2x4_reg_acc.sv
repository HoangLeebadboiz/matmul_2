`timescale 1ns/1ps
`default_nettype none

// C[8][8] = A[8][8] * B[8][8]. All EIGHT output tiles run concurrently.
// Each physical grid owns one 2x4 C tile and ONE shared multiplier/adder.
// Grid array is 4 rows x 2 columns. All eight multipliers prefer hard blocks.
// A tiles multicast across grid rows; B tiles multicast down grid columns.
// 64 partial sums live in registers: 64 x 35 = 2240 bits by default.
// Target: only the 8 input banks use M9K. C uses registers and logic muxes.
// K_TILE=2: A input tile is 2x2; B input tile is 2x4; C output tile is 2x4.
// Output uses an 8-to-1 mux between registered per-grid output values.
// Confirm actual mapping, LE usage and timing in Quartus for EP4CE6E22C8.
module matmul8x8_grid2x4_reg_acc #(
    parameter integer DATA_W = 16,
    parameter integer ACC_W  = 2*DATA_W + 3
) (
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       load_valid,
    output wire                       load_ready,
    input  wire                       load_matrix, // 0=A, 1=B
    input  wire [5:0]                 load_addr,   // row*8+column
    input  wire signed [DATA_W-1:0]   load_data,
    input  wire                       start,
    output wire                       start_ready,
    output wire                       busy,
    output logic                      done,
    output wire                       result_valid,
    input  wire                       result_ready,
    output wire [5:0]                 result_addr, // ordered 0..63
    output wire signed [ACC_W-1:0]    result_data
);
    typedef enum logic [2:0] {
        IDLE, FETCH0, FETCH1, CAPTURE1, ISSUE, DRAIN, WRITE_C
    } state_t;
    state_t state;
    logic [63:0] loaded_a, loaded_b;
    logic [1:0] k_tile;
    logic [5:0] out_index;
    localparam logic [3:0] LAST_ISSUE = 4'd15;
    logic [3:0] issue_index;
    wire wr_fire, wr_a, wr_b;
    wire ram_rd_en, issue, phase, first_step, last_step;
    wire [2:0] read_k;

    wire [2*DATA_W-1:0] qa [0:3]; // low=even row, high=odd row
    wire [2*DATA_W-1:0] qb [0:3]; // low=even col, high=odd col
    // One copy of each input value: A shared across two grid columns,
    // B shared across four grid rows. Buffers hold both k values of a chunk.
    // tile_a[row][phase]: 4 tiles * 4 elements = 16 registers.
    // tile_b[column][phase]: 2 tiles * 8 elements = 16 registers.
    logic signed [DATA_W-1:0] tile_a [0:7][0:1];
    logic signed [DATA_W-1:0] tile_b [0:7][0:1];
    wire signed [DATA_W-1:0] a_bus [0:7];
    wire signed [DATA_W-1:0] b_bus [0:7];
    wire [7:0] grid_done;
    wire signed [ACC_W-1:0] grid_result [0:7];
    wire result_prefetch;
    wire [5:0] prefetch_addr;
    wire [2:0] prefetch_grid, output_grid;
    wire [2:0] prefetch_dest;

    // synthesis translate_off
    initial begin
        if (DATA_W < 8 || DATA_W%8 != 0)
            $fatal(1, "Packed BRAM requires byte-aligned DATA_W, e.g. 8 or 16");
        if (ACC_W < 2*DATA_W + 3)
            $fatal(1, "8x8 signed accumulation requires ACC_W >= 2*DATA_W+3");
    end
    // synthesis translate_on

    assign load_ready = !rst && (state == IDLE);
    // Input load wins if requested at the same time as start.
    assign start_ready = !rst && (state == IDLE) &&
                         (&loaded_a) && (&loaded_b) && !load_valid;
    assign busy = (state != IDLE);
    assign wr_fire = load_valid && load_ready;
    assign wr_a = wr_fire && !load_matrix;
    assign wr_b = wr_fire &&  load_matrix;
    assign ram_rd_en = !rst && ((state == FETCH0) || (state == FETCH1));
    assign read_k = {k_tile, (state == FETCH1)};
    assign issue = !rst && (state == ISSUE);
    assign phase = issue_index[3];
    // This stays high for all eight destinations at k=0.
    assign first_step = (k_tile == 2'd0) && !phase;
    assign last_step = (issue_index == LAST_ISSUE);

    // Capture C[0] on the final DRAIN edge, after the last accumulator update.
    // Thereafter fetch the NEXT scalar only when the current one is accepted.
    // A stalled output causes no new capture: output data/address stay stable.
    assign result_prefetch = !rst &&
        (((state == DRAIN) && (k_tile == 2'd3) && (&grid_done)) ||
         ((state == WRITE_C) && result_ready && (out_index != 6'd63)));
    assign prefetch_addr = (state == WRITE_C) ? out_index + 6'd1 : 6'd0;
    assign prefetch_grid = {prefetch_addr[5:4], prefetch_addr[2]};
    assign prefetch_dest = {prefetch_addr[3], prefetch_addr[1:0]};
    assign output_grid = {out_index[5:4], out_index[2]};

    // Four A banks, one per pair of rows. A tile is packed by COLUMN:
    //   mem[2*t]   = {A[2*i+1][2*t],   A[2*i][2*t]}.
    //   mem[2*t+1] = {A[2*i+1][2*t+1], A[2*i][2*t+1]}.
    // Four B banks, one per pair of columns. A 2x4 B tile spans TWO banks,
    // each providing a pair of columns at k=2*t and k=2*t+1:
    //   mem[2*t]   = {B[2*t][2*j+1],   B[2*t][2*j]}.
    //   mem[2*t+1] = {B[2*t+1][2*j+1], B[2*t+1][2*j]}.
    // Eight bank reads per scalar k return 16 elements, shared by all grids.
    genvar bank, bus_index, grid_row, grid_col;
    generate
        for (bank = 0; bank < 4; bank = bank + 1) begin : g_banks
            tiled_bram_bank #(.DATA_W(DATA_W), .ADDR_W(3)) u_a (
                .clk(clk), .wr_en(wr_a && load_addr[5:4] == bank),
                .wr_lane(load_addr[3]),
                .wr_addr(load_addr[2:0]), .wr_data(load_data),
                .rd_en(ram_rd_en), .rd_addr(read_k), .rd_data(qa[bank])
            );
            tiled_bram_bank #(.DATA_W(DATA_W), .ADDR_W(3)) u_b (
                .clk(clk), .wr_en(wr_b && load_addr[2:1] == bank),
                .wr_lane(load_addr[0]),
                .wr_addr(load_addr[5:3]), .wr_data(load_data),
                .rd_en(ram_rd_en), .rd_addr(read_k), .rd_data(qb[bank])
            );
        end
        for (bus_index = 0; bus_index < 8; bus_index = bus_index + 1) begin : g_bus
            assign a_bus[bus_index] = tile_a[bus_index][phase];
            assign b_bus[bus_index] = tile_b[bus_index][phase];
        end

        for (grid_row = 0; grid_row < 4; grid_row = grid_row + 1) begin : g_grid_row
            for (grid_col = 0; grid_col < 2; grid_col = grid_col + 1) begin : g_grid_col
                // All grid_col values share the same A pair for this row.
                // All grid_row values share the same FOUR B values.
                pe_grid2x4_1mac_reg #(
                    .DATA_W(DATA_W), .ACC_W(ACC_W)
                ) u_grid (
                    .clk(clk), .rst(rst), .step_valid(issue),
                    .first_step(first_step), .last_step(last_step),
                    .dest(issue_index[2:0]),
                    .a0(a_bus[2*grid_row]), .a1(a_bus[2*grid_row+1]),
                    .b0(b_bus[4*grid_col]), .b1(b_bus[4*grid_col+1]),
                    .b2(b_bus[4*grid_col+2]), .b3(b_bus[4*grid_col+3]),
                    .readout_valid(result_prefetch &&
                                   prefetch_grid == 2*grid_row+grid_col),
                    .readout_dest(prefetch_dest),
                    .readout_data(grid_result[2*grid_row+grid_col]),
                    .chunk_done(grid_done[2*grid_row+grid_col])
                );
            end
        end
    endgenerate

    // Accumulators do not change during output. Stream in row-major order.
    assign result_valid = !rst && (state == WRITE_C);
    assign result_addr = out_index;
    assign result_data = grid_result[output_grid];

    always_ff @(posedge clk) begin
        if (rst) begin
            loaded_a <= '0;
            loaded_b <= '0;
        end else if (wr_fire) begin
            if (load_matrix)
                loaded_b[load_addr] <= 1'b1;
            else
                loaded_a[load_addr] <= 1'b1;
        end
    end

    integer n;
    always_ff @(posedge clk) begin
        // Each read has one synchronous RAM cycle before buffer capture.
        if (!rst && state == FETCH1) begin
            for (n = 0; n < 4; n = n + 1) begin
                tile_a[2*n][0]   <= $signed(qa[n][DATA_W-1:0]);
                tile_a[2*n+1][0] <= $signed(qa[n][2*DATA_W-1:DATA_W]);
                tile_b[2*n][0]   <= $signed(qb[n][DATA_W-1:0]);
                tile_b[2*n+1][0] <= $signed(qb[n][2*DATA_W-1:DATA_W]);
            end
        end
        if (!rst && state == CAPTURE1) begin
            for (n = 0; n < 4; n = n + 1) begin
                tile_a[2*n][1]   <= $signed(qa[n][DATA_W-1:0]);
                tile_a[2*n+1][1] <= $signed(qa[n][2*DATA_W-1:DATA_W]);
                tile_b[2*n][1]   <= $signed(qb[n][DATA_W-1:0]);
                tile_b[2*n+1][1] <= $signed(qb[n][2*DATA_W-1:DATA_W]);
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            k_tile <= 2'd0;
            out_index <= 6'd0;
            issue_index <= 4'd0;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: if (start && start_ready) begin
                    k_tile <= 2'd0;
                    out_index <= 6'd0;
                    issue_index <= 4'd0;
                    state <= FETCH0;
                end
                FETCH0:   state <= FETCH1;
                FETCH1:   state <= CAPTURE1;
                CAPTURE1: begin
                    issue_index <= 4'd0;
                    state <= ISSUE;
                end
                ISSUE: begin
                    if (last_step)
                        state <= DRAIN;
                    else
                        issue_index <= issue_index + 4'd1;
                end
                DRAIN: if (&grid_done) begin
                    if (k_tile != 2'd3) begin
                        k_tile <= k_tile + 2'd1;
                        state <= FETCH0;
                    end else begin
                        out_index <= 6'd0;
                        state <= WRITE_C;
                    end
                end
                WRITE_C: if (result_ready) begin
                    if (out_index == 6'd63) begin
                        state <= IDLE;
                        done <= 1'b1;
                    end else begin
                        out_index <= out_index + 6'd1;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
