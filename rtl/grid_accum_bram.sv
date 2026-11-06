`timescale 1ns/1ps
`default_nettype none

// Four partial sums for ONE grid, stored in one M9K at ACC_W=35.
// Simple dual-port: one synchronous read and one write per clock.
// The schedule NEVER reads and writes the same address on the same edge.
// no_rw_check prevents requiring emulation of a collision we never use.
// No RAM/output reset: first_step overwrites every destination at k=0.
module grid_accum_bram #(
    parameter integer ACC_W = 35
) (
    input  wire                     clk,
    input  wire                     rd_en,
    input  wire [1:0]               rd_addr,
    output logic signed [ACC_W-1:0] rd_data,
    input  wire                     wr_en,
    input  wire [1:0]               wr_addr,
    input  wire signed [ACC_W-1:0]  wr_data
);
    (* ramstyle = "M9K, no_rw_check" *) logic signed [ACC_W-1:0] mem [0:3];

    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        if (rd_en)
            rd_data <= mem[rd_addr];
    end
endmodule

`default_nettype wire
