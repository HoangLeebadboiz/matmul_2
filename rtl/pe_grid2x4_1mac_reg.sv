`timescale 1ns/1ps
`default_nettype none

// One hard-multiplier-preferred datapath shared by EIGHT partial sums.
// For each k, dest=0..7 addresses C00,C01,C02,C03,C10,C11,C12,C13.
// first_step must be high for ALL EIGHT destinations at the first k.
// Stage 1 registers product/destination; stage 2 reads a C register through
// a mux, adds the product, and updates that register. C is NOT stored in M9K.
// One read mux is shared between accumulation and registered output capture.
module pe_grid2x4_1mac_reg #(
    parameter integer DATA_W = 16,
    parameter integer ACC_W  = 2*DATA_W + 3
) (
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       step_valid,
    input  wire                       first_step,
    input  wire                       last_step,
    input  wire [2:0]                 dest,
    input  wire signed [DATA_W-1:0]   a0, a1, b0, b1, b2, b3,
    input  wire                       readout_valid,
    input  wire [2:0]                 readout_dest,
    output logic signed [ACC_W-1:0]   readout_data,
    output logic                      chunk_done
);
    localparam integer PROD_W = 2*DATA_W;
    wire signed [DATA_W-1:0] a_selected = dest[2] ? a1 : a0;
    wire [1:0] column_select = dest[1:0];
    logic signed [DATA_W-1:0] b_selected;
    (* multstyle = "dsp" *) wire signed [PROD_W-1:0] product_comb;
    logic signed [PROD_W-1:0] product;
    (* ramstyle = "logic" *) logic signed [ACC_W-1:0] accum [0:7];
    wire signed [ACC_W-1:0] acc_selected, acc_next;
    wire signed [ACC_W-1:0] extended_product;
    logic product_valid, product_first, product_last;
    logic [2:0] product_dest;
    wire [2:0] acc_read_addr = product_valid ? product_dest : readout_dest;
    wire acc_update = !rst && product_valid;

    always_comb begin
        case (column_select)
            2'd0: b_selected = b0;
            2'd1: b_selected = b1;
            2'd2: b_selected = b2;
            2'd3: b_selected = b3;
            default: b_selected = 'x;
        endcase
    end
    // Direct assignment to attributed wire. Confirm actual mapping in Quartus.
    assign product_comb = a_selected * b_selected;

    assign extended_product = {{(ACC_W-PROD_W){product[PROD_W-1]}}, product};
    // Asynchronous register selection; this is an 8-to-1 logic mux.
    assign acc_selected = accum[acc_read_addr];
    assign acc_next = product_first ? extended_product : acc_selected + extended_product;

    integer n;
    always_ff @(posedge clk) begin
        if (rst) begin
            for (n = 0; n < 8; n = n + 1)
                accum[n] <= '0;
        end else if (acc_update) begin
            accum[product_dest] <= acc_next;
        end
    end

    // Output capture keeps the same latency/backpressure contract as BRAM_ACC.
    // Compute and readout are exclusive, enforced by the top and testbench.
    always_ff @(posedge clk) begin
        if (!rst && readout_valid)
            readout_data <= acc_selected;
    end

    always_ff @(posedge clk) begin
        if (step_valid && !rst)
            product <= product_comb;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            product_valid <= 1'b0;
            product_first <= 1'b0;
            product_last <= 1'b0;
            product_dest <= 3'd0;
            chunk_done <= 1'b0;
        end else begin
            product_valid <= step_valid;
            chunk_done <= product_valid && product_last;
            if (step_valid) begin
                product_dest <= dest;
                product_first <= first_step;
                product_last <= last_step;
            end
        end
    end
endmodule

`default_nettype wire
