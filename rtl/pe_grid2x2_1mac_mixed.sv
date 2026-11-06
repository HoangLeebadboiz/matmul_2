`timescale 1ns/1ps
`default_nettype none

// One multiplier/adder shared by FOUR output accumulators.
// For each scalar k, issue dest=0,1,2,3 (C00,C01,C10,C11).
// first_step must be high for ALL FOUR destinations at the first k.
// Stage 1 registers product/destination; stage 2 updates that destination.
// USE_LOGIC_MULT selects a synthesis implementation, NOT extra latency.
// Both branches retain exactly the same product register and accumulator.
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
    output wire signed [ACC_W-1:0]    c00, c01, c10, c11,
    output logic                      chunk_done
);
    localparam integer PROD_W = 2*DATA_W;
    wire signed [DATA_W-1:0] a_selected = dest[1] ? a1 : a0;
    wire signed [DATA_W-1:0] b_selected = dest[0] ? b1 : b0;
    wire signed [PROD_W-1:0] product_comb;
    logic signed [PROD_W-1:0] product;
    logic signed [ACC_W-1:0] c [0:3];
    wire signed [ACC_W-1:0] extended_product;
    logic product_valid, product_first, product_last;
    logic [1:0] product_dest;

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
    assign c00 = c[0];
    assign c01 = c[1];
    assign c10 = c[2];
    assign c11 = c[3];

    always_ff @(posedge clk) begin
        if (step_valid && !rst)
            product <= product_comb;
    end

    integer n;
    always_ff @(posedge clk) begin
        if (rst) begin
            product_valid <= 1'b0;
            product_first <= 1'b0;
            product_last <= 1'b0;
            product_dest <= 2'd0;
            chunk_done <= 1'b0;
            for (n = 0; n < 4; n = n + 1)
                c[n] <= '0;
        end else begin
            product_valid <= step_valid;
            chunk_done <= product_valid && product_last;
            if (step_valid) begin
                product_dest <= dest;
                product_first <= first_step;
                product_last <= last_step;
            end
            if (product_valid)
                c[product_dest] <= product_first ? extended_product :
                                  c[product_dest] + extended_product;
        end
    end
endmodule

`default_nettype wire
