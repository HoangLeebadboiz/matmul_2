`timescale 1ns/1ps
`default_nettype none

module tb_matmul8x8_grid2x4_reg_acc;
    parameter integer DATA_W = 16;
    localparam integer ACC_W = 2*DATA_W + 3;
    localparam integer STEPS_PER_K = 8;
    localparam integer EXPECTED_ISSUES = 8*STEPS_PER_K;
    localparam integer EXPECTED_CYCLES = 4*(3 + 2*STEPS_PER_K + 2) + 64;
    localparam integer VMIN = -(1 << (DATA_W-1));
    localparam integer VMAX =  (1 << (DATA_W-1)) - 1;

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst = 1'b1;
    logic load_valid = 0, load_matrix = 0;
    wire load_ready;
    logic [5:0] load_addr = 0;
    logic signed [DATA_W-1:0] load_data = 0;
    logic start = 0;
    wire start_ready, busy, done;
    wire result_valid;
    logic result_ready = 1;
    wire [5:0] result_addr;
    wire signed [ACC_W-1:0] result_data;

    matmul8x8_grid2x4_reg_acc #(.DATA_W(DATA_W), .ACC_W(ACC_W)) dut (.*);

    // Simulation-only visibility of all 64 accumulator registers.
    wire signed [ACC_W-1:0] accum_view [0:63];
    genvar grid_r, grid_c, local_slot;
    generate
        for (grid_r = 0; grid_r < 4; grid_r = grid_r + 1) begin : g_style_row
            for (grid_c = 0; grid_c < 2; grid_c = grid_c + 1) begin : g_style_col
                for (local_slot = 0; local_slot < 8; local_slot = local_slot + 1) begin : g_acc_view
                    assign accum_view[(2*grid_r+local_slot/4)*8+4*grid_c+local_slot%4] =
                        dut.g_grid_row[grid_r].g_grid_col[grid_c].u_grid.accum[local_slot];
                    always @(posedge clk) begin
                        if (rst) begin
                            #1;
                            if (accum_view[(2*grid_r+local_slot/4)*8+4*grid_c+local_slot%4] !== {ACC_W{1'b0}})
                                $fatal(1, "Accumulator register did not reset to zero");
                        end
                    end
                end

                wire compute_step = dut.g_grid_row[grid_r].g_grid_col[grid_c].u_grid.step_valid;
                wire output_capture = dut.g_grid_row[grid_r].g_grid_col[grid_c].u_grid.readout_valid;
                wire acc_update = dut.g_grid_row[grid_r].g_grid_col[grid_c].u_grid.acc_update;
                wire first_product = dut.g_grid_row[grid_r].g_grid_col[grid_c].u_grid.product_first;
                wire [2:0] update_addr = dut.g_grid_row[grid_r].g_grid_col[grid_c].u_grid.product_dest;
                integer compute_steps = 0, output_captures = 0, acc_updates = 0;
                logic [7:0] initialized = 0;
                always @(posedge clk) begin
                    if (rst || (start && start_ready)) begin
                        compute_steps = 0;
                        output_captures = 0;
                        acc_updates = 0;
                        initialized = 0;
                    end else begin
                        if (output_capture && (compute_step || acc_update))
                            $fatal(1, "Compute/output shared the accumulator read mux");
                        if (compute_step)
                            compute_steps = compute_steps + 1;
                        if (output_capture) begin
                            if (initialized !== 8'hff)
                                $fatal(1, "Output captured before all sums were initialized");
                            output_captures = output_captures + 1;
                        end
                        if (acc_update) begin
                            if (!first_product && !initialized[update_addr])
                                $fatal(1, "Accumulation used a stale sum from a previous run");
                            initialized[update_addr] = 1'b1;
                            acc_updates = acc_updates + 1;
                        end
                    end
                end
                always @(negedge clk) begin
                    if (!rst && done) begin
                        if (compute_steps != 64 || acc_updates != 64 || output_captures != 8)
                            $fatal(1, "G%0d%0d unexpected register counts %0d/%0d/%0d",
                                   grid_r, grid_c, compute_steps, acc_updates, output_captures);
                    end
                end
            end
        end
    endgenerate

    logic signed [DATA_W-1:0] a [0:63];
    logic signed [DATA_W-1:0] b [0:63];
    longint signed golden [0:63];
    logic [31:0] rng = 32'h4c2a917d;
    integer pass_count = 0;
    integer issue_count = 0, read_count = 0, active_cycles = 0;

    // Independent reference for EVERY output after each scalar K step.
    integer issued_steps = 0;
    integer pending_k, pending_dest, pending_step, mon_k, mon_n, mon_dest;
    bit pending_valid = 0, accepted_step;
    logic signed [ACC_W-1:0] old_accum [0:63];
    logic [7:0] seen_a [0:3];
    logic [7:0] seen_b [0:3];
    integer reads_a [0:3];
    integer reads_b [0:3];
    // Simulation-only visibility for independently checking the tile layout.
    wire [2*DATA_W-1:0] stored_a [0:3][0:7];
    wire [2*DATA_W-1:0] stored_b [0:3][0:7];

    function automatic longint signed partial_sum(input integer row,
                                                  input integer col,
                                                  input integer last_k);
        integer k;
        begin
            partial_sum = 0;
            for (k = 0; k <= last_k; k = k + 1)
                partial_sum = partial_sum + longint'(a[8*row+k]) * longint'(b[8*k+col]);
        end
    endfunction

    // Observe the actual read port of each physical bank instance. Require
    // every input address to be read once, in order, per matrix product.
    genvar bank, word_index;
    generate
        for (bank = 0; bank < 4; bank = bank + 1) begin : g_read_monitor
            for (word_index = 0; word_index < 8; word_index = word_index + 1) begin : g_storage_view
                assign stored_a[bank][word_index] = dut.g_banks[bank].u_a.mem[word_index];
                assign stored_b[bank][word_index] = dut.g_banks[bank].u_b.mem[word_index];
            end

            // Check EVERY scalar write: the other half-word must retain
            // its exact previous bits, including X before initialization.
            integer written_addr_a, written_addr_b;
            logic [2*DATA_W-1:0] expected_word_a, expected_word_b;
            always @(posedge clk) begin
                if (dut.g_banks[bank].u_a.wr_en) begin
                    if (dut.g_banks[bank].u_a.rd_en)
                        $fatal(1, "Unexpected concurrent A load/read");
                    written_addr_a = dut.g_banks[bank].u_a.wr_addr;
                    expected_word_a = stored_a[bank][written_addr_a];
                    expected_word_a[dut.g_banks[bank].u_a.wr_lane*DATA_W +: DATA_W] = load_data;
                    #1;
                    if (stored_a[bank][written_addr_a] !== expected_word_a)
                        $fatal(1, "A bank %0d write corrupted a packed word", bank);
                end
            end
            always @(posedge clk) begin
                if (dut.g_banks[bank].u_b.wr_en) begin
                    if (dut.g_banks[bank].u_b.rd_en)
                        $fatal(1, "Unexpected concurrent B load/read");
                    written_addr_b = dut.g_banks[bank].u_b.wr_addr;
                    expected_word_b = stored_b[bank][written_addr_b];
                    expected_word_b[dut.g_banks[bank].u_b.wr_lane*DATA_W +: DATA_W] = load_data;
                    #1;
                    if (stored_b[bank][written_addr_b] !== expected_word_b)
                        $fatal(1, "B bank %0d write corrupted a packed word", bank);
                end
            end

            always @(posedge clk) begin
                if (rst || (start && start_ready)) begin
                    seen_a[bank] <= '0;
                    seen_b[bank] <= '0;
                    reads_a[bank] <= 0;
                    reads_b[bank] <= 0;
                end else begin
                    if (dut.g_banks[bank].u_a.rd_en) begin
                        if (!busy || seen_a[bank][dut.g_banks[bank].u_a.rd_addr] ||
                            dut.g_banks[bank].u_a.rd_addr !== reads_a[bank])
                            $fatal(1, "Duplicate/unexpected A bank %0d read", bank);
                        seen_a[bank][dut.g_banks[bank].u_a.rd_addr] <= 1'b1;
                        reads_a[bank] <= reads_a[bank] + 1;
                    end
                    if (dut.g_banks[bank].u_b.rd_en) begin
                        if (!busy || seen_b[bank][dut.g_banks[bank].u_b.rd_addr] ||
                            dut.g_banks[bank].u_b.rd_addr !== reads_b[bank])
                            $fatal(1, "Duplicate/unexpected B bank %0d read", bank);
                        seen_b[bank][dut.g_banks[bank].u_b.rd_addr] <= 1'b1;
                        reads_b[bank] <= reads_b[bank] + 1;
                    end
                end
            end
        end
    endgenerate

    always @(posedge clk) begin
        if (rst || (start && start_ready)) begin
            pending_valid = 0;
            issued_steps = 0;
        end else begin
            accepted_step = dut.issue;
            if (accepted_step) begin
                mon_k = issued_steps/STEPS_PER_K;
                mon_dest = issued_steps%STEPS_PER_K;
                if (mon_k >= 8 || dut.k_tile !== mon_k/2 ||
                    dut.first_step !== (mon_k == 0) || dut.phase !== (mon_k%2 == 1))
                    $fatal(1, "Incorrect first_step/phase/K at issue %0d", mon_k);
                if (dut.last_step !== (issued_steps%(2*STEPS_PER_K) == 2*STEPS_PER_K-1))
                    $fatal(1, "Incorrect last_step at issue %0d", issued_steps);
                if (dut.issue_index[2:0] !== mon_dest)
                    $fatal(1, "Incorrect shared-MAC destination at issue %0d", issued_steps);
                for (mon_n = 0; mon_n < 8; mon_n = mon_n + 1) begin
                    if (dut.a_bus[mon_n] !== a[8*mon_n+mon_k] ||
                        dut.b_bus[mon_n] !== b[8*mon_k+mon_n])
                        $fatal(1, "Wrong shared A/B data at index=%0d k=%0d", mon_n, mon_k);
                end
            end
            for (mon_n = 0; mon_n < 64; mon_n = mon_n + 1)
                old_accum[mon_n] = accum_view[mon_n];
            #1;
            // One local destination updates in each of the eight grids.
            for (mon_n = 0; mon_n < 64; mon_n = mon_n + 1) begin
                if (pending_valid &&
                    4*((mon_n/8)%2) + mon_n%4 == pending_dest) begin
                    if (longint'(accum_view[mon_n]) !== partial_sum(mon_n/8, mon_n%8, pending_k))
                        $fatal(1, "Partial C[%0d,%0d] wrong after k=%0d", mon_n/8, mon_n%8, pending_k);
                end else if (accum_view[mon_n] !== old_accum[mon_n])
                    $fatal(1, "Inactive accumulator changed at C[%0d]", mon_n);
            end
            if (dut.grid_done !== {8{pending_valid &&
                (pending_step%(2*STEPS_PER_K) == 2*STEPS_PER_K-1)}})
                $fatal(1, "All 8 chunk_done flags must track the same last step");
            pending_valid = accepted_step;
            if (accepted_step) begin
                pending_k = mon_k;
                pending_dest = mon_dest;
                pending_step = issued_steps;
                issued_steps = issued_steps + 1;
            end
        end
    end

    function automatic [31:0] next_random;
        begin
            rng = rng ^ (rng << 13);
            rng = rng ^ (rng >> 17);
            rng = rng ^ (rng << 5);
            next_random = rng;
        end
    endfunction

    // Count actual parallel work and source reads, independently of FSM timing.
    always @(posedge clk) begin
        if (rst || (start && start_ready)) begin
            issue_count = 0;
            read_count = 0;
            active_cycles = 0;
        end else if (busy) begin
            active_cycles = active_cycles + 1;
            if (dut.issue) issue_count = issue_count + 1;
            if (dut.ram_rd_en) read_count = read_count + 1;
        end
    end

    task automatic reset_core;
        begin
            @(negedge clk);
            rst = 1;
            start = 0;
            load_valid = 0;
            result_ready = 1;
            repeat (3) @(negedge clk);
            rst = 0;
            @(negedge clk);
            if (busy || result_valid || done || start_ready)
                $fatal(1, "Reset did not clear controller/input-valid masks");
        end
    endtask

    task automatic write_element(input bit matrix_sel, input integer addr,
                                 input logic signed [DATA_W-1:0] data);
        begin
            @(negedge clk);
            load_valid = 1;
            load_matrix = matrix_sel;
            load_addr = addr[5:0];
            load_data = data;
            @(posedge clk);
            if (!load_ready) $fatal(1, "Input write was not accepted");
        end
    endtask

    task automatic load_matrices;
        integer n, addr;
        begin
            // A permutation of all addresses catches assumptions about
            // strictly ordered loading. Interleave A and B writes.
            for (n = 0; n < 64; n = n + 1) begin
                addr = (5*n + 3) % 64;
                write_element(0, addr, a[addr]);
                write_element(1, addr, b[addr]);
            end
            @(negedge clk);
            load_valid = 0;
        end
    endtask

    task automatic check_storage;
        integer bank_id, k;
        logic [2*DATA_W-1:0] expected_a, expected_b;
        begin
            for (bank_id = 0; bank_id < 4; bank_id = bank_id + 1)
                for (k = 0; k < 8; k = k + 1) begin
                    expected_a = {a[8*(2*bank_id+1)+k], a[8*(2*bank_id)+k]};
                    expected_b = {b[8*k+2*bank_id+1], b[8*k+2*bank_id]};
                    if (stored_a[bank_id][k] !== expected_a || stored_b[bank_id][k] !== expected_b)
                        $fatal(1, "Tile packing wrong: bank=%0d word=%0d", bank_id, k);
                end
        end
    endtask

    // Example host-side tile loader using the unchanged scalar interface.
    task automatic load_tile(input bit matrix_sel, input integer tile_row,
                              input integer tile_col);
        integer r, c, addr, tile_width;
        begin
            tile_width = matrix_sel ? 4 : 2;
            for (r = 0; r < 2; r = r + 1)
                for (c = 0; c < tile_width; c = c + 1) begin
                    addr = 8*(2*tile_row+r) + tile_width*tile_col+c;
                    write_element(matrix_sel, addr, matrix_sel ? b[addr] : a[addr]);
                end
        end
    endtask

    task automatic load_matrices_tiled;
        integer tile_row, tile_col;
        begin
            for (tile_row = 0; tile_row < 4; tile_row = tile_row + 1)
                for (tile_col = 0; tile_col < 4; tile_col = tile_col + 1)
                    load_tile(0, tile_row, tile_col);
            for (tile_row = 0; tile_row < 4; tile_row = tile_row + 1)
                for (tile_col = 0; tile_col < 2; tile_col = tile_col + 1)
                    load_tile(1, tile_row, tile_col);
            @(negedge clk);
            load_valid = 0;
        end
    endtask

    task automatic patch_element(input bit matrix_sel, input integer addr,
                                  input logic signed [DATA_W-1:0] value);
        begin
            if (matrix_sel) b[addr] = value;
            else            a[addr] = value;
            write_element(matrix_sel, addr, value);
            @(negedge clk);
            load_valid = 0;
        end
    endtask

    task automatic compute_golden;
        integer i, j, k;
        longint signed sum;
        begin
            for (i = 0; i < 8; i = i + 1)
                for (j = 0; j < 8; j = j + 1) begin
                    sum = 0;
                    for (k = 0; k < 8; k = k + 1)
                        sum = sum + longint'(a[8*i+k]) * longint'(b[8*k+j]);
                    golden[8*i+j] = sum;
                end
        end
    endtask

    task automatic run_case(input string label_text, input bit reload,
                            input bit stalls, input bit inject_busy_requests);
        integer count, cycles, forced_stall, final_stall, expected_addr, bank_index;
        logic [63:0] seen;
        bit previous_stall, finished;
        logic [5:0] held_addr;
        logic signed [ACC_W-1:0] held_data;
        logic [31:0] rv;
        begin
            compute_golden();
            if (reload) load_matrices();
            check_storage();
            @(negedge clk);
            start = 1;
            @(posedge clk);
            if (!start_ready) $fatal(1, "start_ready low for %s", label_text);
            @(negedge clk);
            start = 0;
            count = 0;
            cycles = 0;
            forced_stall = 0;
            final_stall = 0;
            seen = '0;
            previous_stall = 0;
            finished = 0;
            while (!finished) begin
                // Drive at falling edges; check the handshake at rising edges.
                rv = next_random();
                result_ready = !stalls || (rv[1:0] != 0);
                if (stalls && result_valid && count == 0 && forced_stall < 8) begin
                    result_ready = 0;
                    forced_stall = forced_stall + 1;
                end
                if (stalls && result_valid && count == 63 && final_stall < 8) begin
                    result_ready = 0;
                    final_stall = final_stall + 1;
                end
                // Pulses while busy must neither overwrite RAM nor restart C.
                load_valid = inject_busy_requests && cycles >= 5 && cycles <= 7;
                start = load_valid;
                load_matrix = 0;
                load_addr = 0;
                load_data = VMAX;

                @(posedge clk);
                if (load_ready || start_ready)
                    $fatal(1, "Ready asserted while processing %s", label_text);
                if (previous_stall &&
                    (!result_valid || result_addr !== held_addr || result_data !== held_data))
                    $fatal(1, "Output changed during backpressure: %s", label_text);
                if (result_valid && result_ready) begin
                    // Every tile finishes together; serialize C row-major.
                    expected_addr = count;
                    if (count >= 64 || result_addr !== expected_addr || seen[result_addr])
                        $fatal(1, "Output address/order mismatch: got %0d expected %0d", result_addr, expected_addr);
                    if (result_data !== golden[result_addr][ACC_W-1:0])
                        $fatal(1, "%s C[%0d]: got %0d expected %0d", label_text,
                               result_addr, $signed(result_data), golden[result_addr]);
                    seen[result_addr] = 1'b1;
                    count = count + 1;
                end
                previous_stall = result_valid && !result_ready;
                held_addr = result_addr;
                held_data = result_data;
                #1;
                if (done) begin
                    if (count != 64 || !(&seen) || busy || result_valid)
                        $fatal(1, "Incorrect completion handshake: %s", label_text);
                    if (issue_count != EXPECTED_ISSUES || read_count != 8)
                        $fatal(1, "Expected %0d issue and 8 RAM-read cycles, got %0d/%0d", EXPECTED_ISSUES, issue_count, read_count);
                    if (!stalls && active_cycles != EXPECTED_CYCLES)
                        $fatal(1, "Expected %0d no-stall cycles, got %0d", EXPECTED_CYCLES, active_cycles);
                    for (bank_index = 0; bank_index < 4; bank_index = bank_index + 1) begin
                        if (reads_a[bank_index] != 8 || reads_b[bank_index] != 8 ||
                            seen_a[bank_index] !== 8'hff || seen_b[bank_index] !== 8'hff)
                            $fatal(1, "Inputs were not read exactly once in bank %0d", bank_index);
                    end
                    finished = 1;
                end
                cycles = cycles + 1;
                if (cycles > 1000) $fatal(1, "Timeout: %s", label_text);
                @(negedge clk);
            end
            start = 0;
            load_valid = 0;
            result_ready = 1;
            pass_count = pass_count + 1;
            if (pass_count <= 8)
                $display("PASS %-24s cycles=%0d issue=%0d word_reads=%0d scalar_reads=%0d", label_text,
                         active_cycles, issue_count, read_count*8, read_count*16);
            @(posedge clk);
            #1;
            if (done) $fatal(1, "done must be a one-cycle pulse");
        end
    endtask

    integer n, trial, lane;
    logic [31:0] random_word;
    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile($sformatf("build/matmul8x8_grid2x4_reg_acc_dw%0d.vcd", DATA_W));
            $dumpvars(0, tb_matmul8x8_grid2x4_reg_acc);
        end
        #1;
        $display("RTL CONFIG: 8 concurrent 2x4 grids, one dsp-preferred multiplier/grid (not a physical mapping report)");
        reset_core();

        // Starting with missing input elements must have no effect.
        @(negedge clk); start = 1;
        repeat (3) begin
            @(posedge clk); #1;
            if (busy || start_ready) $fatal(1, "Started before full A/B load");
            @(negedge clk);
        end
        start = 0;

        // A single missing element must block start, including the final
        // cycle that writes it. Simultaneous load/start gives load priority.
        for (n = 0; n < 64; n = n + 1) begin a[n] = 0; b[n] = 0; end
        for (n = 0; n < 64; n = n + 1) write_element(0, n, a[n]);
        for (n = 0; n < 63; n = n + 1) write_element(1, n, b[n]);
        @(negedge clk); load_valid = 0; start = 1;
        @(posedge clk); #1;
        if (busy || start_ready) $fatal(1, "Started with B[63] missing");
        write_element(1, 63, b[63]);
        #1;
        if (busy || start_ready) $fatal(1, "Start overrode the final input load");
        @(negedge clk); load_valid = 0; start = 0;

        for (n = 0; n < 64; n = n + 1) begin a[n] = 0; b[n] = n-8; end
        run_case("zero", 1, 0, 0);
        for (n = 0; n < 64; n = n + 1) begin
            a[n] = (n/8 == n%8) ? 1 : 0;
            b[n] = 3*n - 20;
        end
        run_case("identity_signed", 1, 0, 1);
        run_case("reuse_without_reload", 0, 1, 1);
        for (n = 0; n < 64; n = n + 1) begin a[n] = VMIN; b[n] = VMIN; end
        run_case("minimum_times_minimum", 1, 1, 0);
        for (n = 0; n < 64; n = n + 1) begin a[n] = VMAX; b[n] = VMAX; end
        run_case("maximum_times_maximum", 1, 0, 0);
        for (n = 0; n < 64; n = n + 1) begin a[n] = VMIN; b[n] = VMAX; end
        run_case("minimum_times_maximum", 1, 1, 0);
        for (n = 0; n < 64; n = n + 1) begin
            a[n] = n%2 ? VMIN : VMAX;
            b[n] = (n/8)%2 ? VMAX : VMIN;
        end
        run_case("mixed_extremes", 1, 1, 1);

        // Only G31 can have nonzero outputs. Exercise all eight of its
        // destinations with mixed signs and full-width operands.
        for (n = 0; n < 64; n = n + 1) begin
            a[n] = (n/8 >= 6) ? ((n%3 == 0) ? VMIN : VMAX) : 0;
            b[n] = (n%8 >= 4) ? ((n%5 == 0) ? VMIN : VMAX) : 0;
        end
        run_case("last_grid_only", 1, 0, 0);

        // Isolate every K lane, especially the new k=4..7. Nonzero data
        // in every row/column also exercises all 8 grids/all banks.
        for (lane = 0; lane < 8; lane = lane + 1) begin
            for (n = 0; n < 64; n = n + 1) begin
                a[n] = (n%8 == lane) ? n/8-3 : 0;
                b[n] = (n/8 == lane) ? 2*(n%8)-7 : 0;
            end
            run_case($sformatf("isolated_k_%0d", lane), 1, lane%2, 0);
        end
        for (n = 0; n < 64; n = n + 1) begin
            a[n] = 2*n-63;
            b[n] = (n/8 == n%8) ? 1 : 0;
        end
        run_case("right_identity", 1, 0, 0);

        // Explicitly load A tiles 2x2 and B tiles 2x4 through scalar writes.
        for (n = 0; n < 64; n = n + 1) begin
            a[n] = 7*n-101;
            b[n] = 3*n-87;
        end
        load_matrices_tiled();
        run_case("tile_ordered_load", 0, 0, 0);

        // Change only one half-word, rerun without reloading other values.
        patch_element(0, 8*2+5, VMIN);
        run_case("patch_A_low_lane", 0, 0, 0);
        patch_element(0, 8*3+5, VMAX);
        run_case("patch_A_high_lane", 0, 1, 0);
        patch_element(1, 8*6+4, VMIN);
        run_case("patch_B_low_lane", 0, 0, 0);
        patch_element(1, 8*6+5, VMAX);
        run_case("patch_B_high_lane", 0, 1, 0);

        for (trial = 0; trial < 100; trial = trial + 1) begin
            for (n = 0; n < 64; n = n + 1) begin
                random_word = next_random(); a[n] = random_word[DATA_W-1:0];
                random_word = next_random(); b[n] = random_word[DATA_W-1:0];
            end
            run_case($sformatf("random_%0d", trial), 1, trial%2, trial%3 == 0);
        end

        // Abort in-flight work, reset the load masks, then recover normally.
        @(negedge clk); start = 1;
        @(posedge clk);
        if (!start_ready) $fatal(1, "Cannot start reset-abort test");
        @(negedge clk); start = 0;
        repeat (10) @(negedge clk);
        reset_core();
        run_case("recovery_after_reset", 1, 1, 1);

        // Reset while a valid output is stalled, then fully reload/restart.
        @(negedge clk); start = 1; result_ready = 0;
        @(posedge clk);
        if (!start_ready) $fatal(1, "Cannot start output-reset test");
        @(negedge clk); start = 0;
        while (!result_valid) @(negedge clk);
        repeat (3) @(negedge clk);
        reset_core();
        run_case("recovery_output_reset", 1, 0, 0);

        if (pass_count != 124) $fatal(1, "Unexpected number of completed cases");
        $display("ACCUM REG PASS: per grid/run 64 issues, 64 register updates, 8 output captures; synchronous reset verified");
        $display("ALL PASS: %0d matrix products, DATA_W=%0d ACC_W=%0d, 8 concurrent 2x4 grids (8 dsp-preferred multipliers)", pass_count, DATA_W, ACC_W);
        $finish;
    end

    initial begin
        #2000000;
        $fatal(1, "Global simulation timeout");
    end
endmodule

`default_nettype wire
