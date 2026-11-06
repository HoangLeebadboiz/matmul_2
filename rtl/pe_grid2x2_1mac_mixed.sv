`timescale 1ns/1ps
`default_nettype none

// One multiplier/adder shared by FOUR partial sums stored in one M9K.
// For each scalar k, issue dest=0,1,2,3 (C00,C01,C10,C11).
// first_step must be high for ALL FOUR destinations at the first k.
// Stage 1 registers product/destination AND synchronously reads old C.
// Stage 2 adds old C to product and writes that destination to RAM.
// USE_LOGIC_MULT selects a synthesis implementation, NOT extra latency.
// Both branches retain exactly the same product register and RAM schedule.
module pe_grid2x2_1mac_mixed #(
    parameter integer DATA_W = 16,
    parameter integer ACC_W  = 2*DATA_W + 3,
    parameter bit USE_LOGIC_MULT = 1'b0
) (
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       step_valid,
    input  wire                       first_step,
    input  wire                       last_step,
    input  wire [1:0]                 dest,
    input  wire signed [DATA_W-1:0]   a0, a1, b0, b1,
    input  wire                       readout_valid,
    input  wire [1:0]                 readout_dest,
    output wire signed [ACC_W-1:0]    readout_data,
    output logic                      chunk_done
);
    localparam integer PROD_W = 2*DATA_W;
    wire signed [DATA_W-1:0] a_selected = dest[1] ? a1 : a0;
    wire signed [DATA_W-1:0] b_selected = dest[0] ? b1 : b0;
    wire signed [PROD_W-1:0] product_comb;
    logic signed [PROD_W-1:0] product;
    wire signed [ACC_W-1:0] acc_q, acc_next;
    wire signed [ACC_W-1:0] extended_product;
    logic product_valid, product_first, product_last;
    logic [1:0] product_dest;
    wire acc_rd_en = !rst && (step_valid || readout_valid);
    wire [1:0] acc_rd_addr = step_valid ? dest : readout_dest;
    wire acc_wr_en = !rst && product_valid;

    // Quartus applies multstyle to the multiplication directly assigned to
    // the attributed variable. Literal attributes avoid string parameters.
    // "dsp" expresses a mapping preference; inspect the synthesis report.
    generate
        if (USE_LOGIC_MULT) begin : g_logic
            (* multstyle = "logic" *) wire signed [PROD_W-1:0] mult_result;
            assign mult_result = a_selected * b_selected;
            assign product_comb = mult_result;
        end else begin : g_dsp
            (* multstyle = "dsp" *) wire signed [PROD_W-1:0] mult_result;
            assign mult_result = a_selected * b_selected;
            assign product_comb = mult_result;
        end
    endgenerate

    assign extended_product = {{(ACC_W-PROD_W){product[PROD_W-1]}}, product};
    assign acc_next = product_first ? extended_product : acc_q + extended_product;
    assign readout_data = acc_q;

    grid_accum_bram #(.ACC_W(ACC_W)) u_accum (
        .clk(clk), .rd_en(acc_rd_en), .rd_addr(acc_rd_addr), .rd_data(acc_q),
        .wr_en(acc_wr_en), .wr_addr(product_dest), .wr_data(acc_next)
    );

    always_ff @(posedge clk) begin
        if (step_valid && !rst)
            product <= product_comb;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            product_valid <= 1'b0;
            product_first <= 1'b0;
            product_last <= 1'b0;
            product_dest <= 2'd0;
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
