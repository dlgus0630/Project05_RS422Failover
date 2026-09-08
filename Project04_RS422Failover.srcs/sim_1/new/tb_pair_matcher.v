`timescale 1ns / 1ps

// pair_matcher final-interface self-checking testbench.
// Equal/mismatched pairs, sequence ordering, ambiguous sequence distance,
// runtime timeout selection, consume holdoff, and synchronous clear are checked.

module tb_pair_matcher;

    localparam [1:0] NONE     = 2'b00;
    localparam [1:0] PAIR     = 2'b01;
    localparam [1:0] SINGLE_A = 2'b10;
    localparam [1:0] SINGLE_B = 2'b11;

    reg clk;
    reg reset_p;
    reg clear;
    reg [31:0] pair_timeout_cycles;

    reg         a_empty;
    reg [7:0]   a_frame_length;
    reg [7:0]   a_device_id;
    reg [7:0]   a_command;
    reg [7:0]   a_sequence;
    reg [127:0] a_payload_data;
    reg [15:0]  a_received_crc;
    reg         a_seq_gap;
    wire        pop_a;

    reg         b_empty;
    reg [7:0]   b_frame_length;
    reg [7:0]   b_device_id;
    reg [7:0]   b_command;
    reg [7:0]   b_sequence;
    reg [127:0] b_payload_data;
    reg [15:0]  b_received_crc;
    reg         b_seq_gap;
    wire        pop_b;

    wire         result_valid;
    wire [1:0]   result_kind;
    wire         result_pair_equal;
    wire [5:0]   mismatch_flags;
    wire         result_timeout;
    wire         result_seq_skew;
    wire         result_seq_ambiguous;
    wire         result_a_seq_gap;
    wire         result_b_seq_gap;
    wire         pair_wait_active;
    wire [7:0]   out_sequence;
    wire         out_seq_gap;

    integer error_count;

    pair_matcher #(
        .PAIR_TIMEOUT_CYCLES (3)
    ) dut (
        .clk                  (clk),
        .reset_p              (reset_p),
        .clear                (clear),
        .pair_timeout_cycles  (pair_timeout_cycles),

        .a_empty              (a_empty),
        .a_frame_length       (a_frame_length),
        .a_device_id          (a_device_id),
        .a_command            (a_command),
        .a_sequence           (a_sequence),
        .a_payload_data       (a_payload_data),
        .a_received_crc       (a_received_crc),
        .a_seq_gap            (a_seq_gap),
        .pop_a                (pop_a),

        .b_empty              (b_empty),
        .b_frame_length       (b_frame_length),
        .b_device_id          (b_device_id),
        .b_command            (b_command),
        .b_sequence           (b_sequence),
        .b_payload_data       (b_payload_data),
        .b_received_crc       (b_received_crc),
        .b_seq_gap            (b_seq_gap),
        .pop_b                (pop_b),

        .result_valid         (result_valid),
        .result_kind          (result_kind),
        .result_pair_equal    (result_pair_equal),
        .mismatch_flags       (mismatch_flags),
        .result_timeout       (result_timeout),
        .result_seq_skew      (result_seq_skew),
        .result_seq_ambiguous (result_seq_ambiguous),
        .result_a_seq_gap     (result_a_seq_gap),
        .result_b_seq_gap     (result_b_seq_gap),
        .pair_wait_active     (pair_wait_active),

        .out_frame_length     (),
        .out_device_id        (),
        .out_command          (),
        .out_sequence         (out_sequence),
        .out_payload_data     (),
        .out_received_crc     (),
        .out_seq_gap          (out_seq_gap)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task apply_reset;
        begin
            a_empty  = 1'b1;
            b_empty  = 1'b1;
            a_seq_gap = 1'b0;
            b_seq_gap = 1'b0;
            reset_p  = 1'b1;
            clear    = 1'b0;
            repeat (2) @(posedge clk);
            @(negedge clk);
            reset_p = 1'b0;
        end
    endtask

    task set_empty;
        begin
            a_empty = 1'b1;
            b_empty = 1'b1;
        end
    endtask

    task check_result;
        input       expected_valid;
        input [1:0] expected_kind;
        input       expected_pop_a;
        input       expected_pop_b;
        input       expected_equal;
        input [5:0] expected_mismatch;
        input       expected_timeout;
        input       expected_skew;
        input       expected_ambiguous;
        input       expected_wait;
        input [7:0] expected_sequence;
        begin
            if ((result_valid         !== expected_valid)      ||
                (result_kind          !== expected_kind)       ||
                (pop_a                !== expected_pop_a)      ||
                (pop_b                !== expected_pop_b)      ||
                (result_pair_equal    !== expected_equal)      ||
                (mismatch_flags       !== expected_mismatch)   ||
                (result_timeout       !== expected_timeout)    ||
                (result_seq_skew      !== expected_skew)       ||
                (result_seq_ambiguous !== expected_ambiguous)  ||
                (pair_wait_active     !== expected_wait)       ||
                (expected_valid &&
                 (out_sequence        !== expected_sequence))) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] t=%0t valid=%b/%b kind=%b/%b pop=%b%b/%b%b mismatch=%b/%b timeout=%b/%b skew=%b/%b ambiguous=%b/%b wait=%b/%b seq=%h/%h",
                    $time,
                    result_valid, expected_valid,
                    result_kind, expected_kind,
                    pop_a, pop_b, expected_pop_a, expected_pop_b,
                    mismatch_flags, expected_mismatch,
                    result_timeout, expected_timeout,
                    result_seq_skew, expected_skew,
                    result_seq_ambiguous, expected_ambiguous,
                    pair_wait_active, expected_wait,
                    out_sequence, expected_sequence
                );
            end
        end
    endtask

    task check_gap_metadata;
        input expected_a_seq_gap;
        input expected_b_seq_gap;
        input expected_out_seq_gap;
        begin
            if ((result_a_seq_gap !== expected_a_seq_gap) ||
                (result_b_seq_gap !== expected_b_seq_gap) ||
                (out_seq_gap      !== expected_out_seq_gap)) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] t=%0t gap metadata a=%b/%b b=%b/%b out=%b/%b",
                    $time,
                    result_a_seq_gap, expected_a_seq_gap,
                    result_b_seq_gap, expected_b_seq_gap,
                    out_seq_gap, expected_out_seq_gap
                );
            end
        end
    endtask

    task check_after_clock;
        input       expected_valid;
        input [1:0] expected_kind;
        input       expected_pop_a;
        input       expected_pop_b;
        input       expected_equal;
        input [5:0] expected_mismatch;
        input       expected_timeout;
        input       expected_skew;
        input       expected_ambiguous;
        input       expected_wait;
        input [7:0] expected_sequence;
        begin
            @(posedge clk);
            #1;
            check_result(
                expected_valid,
                expected_kind,
                expected_pop_a,
                expected_pop_b,
                expected_equal,
                expected_mismatch,
                expected_timeout,
                expected_skew,
                expected_ambiguous,
                expected_wait,
                expected_sequence
            );
        end
    endtask

    initial begin
        error_count          = 0;
        reset_p              = 1'b1;
        clear                = 1'b0;
        pair_timeout_cycles  = 32'd3;

        a_empty              = 1'b1;
        a_frame_length       = 8'd7;
        a_device_id          = 8'h21;
        a_command            = 8'h31;
        a_sequence           = 8'h10;
        a_payload_data       = 128'h0000000000000000000000000000A55A;
        a_received_crc       = 16'h1234;
        a_seq_gap            = 1'b0;

        b_empty              = 1'b1;
        b_frame_length       = 8'd7;
        b_device_id          = 8'h21;
        b_command            = 8'h31;
        b_sequence           = 8'h10;
        b_payload_data       = 128'h0000000000000000000000000000A55A;
        b_received_crc       = 16'h1234;
        b_seq_gap            = 1'b0;

        // Equal pair and one-cycle consume holdoff.
        apply_reset;
        @(negedge clk);
        a_seq_gap = 1'b1;
        a_empty = 1'b0;
        b_empty = 1'b0;
        check_after_clock(
            1'b1, PAIR, 1'b1, 1'b1, 1'b1, 6'b000000,
            1'b0, 1'b0, 1'b0, 1'b0, 8'h10
        );
        check_gap_metadata(1'b1, 1'b0, 1'b1);
        check_after_clock(
            1'b0, NONE, 1'b0, 1'b0, 1'b0, 6'b000000,
            1'b0, 1'b0, 1'b0, 1'b0, 8'h00
        );
        @(negedge clk);
        set_empty;

        // Same sequence with payload mismatch.
        apply_reset;
        @(negedge clk);
        a_empty        = 1'b0;
        b_empty        = 1'b0;
        b_payload_data = 128'h0000000000000000000000000000BEEF;
        check_after_clock(
            1'b1, PAIR, 1'b1, 1'b1, 1'b0, 6'b000100,
            1'b0, 1'b0, 1'b0, 1'b0, 8'h10
        );

        // A is older in modulo-256 sequence order.
        apply_reset;
        @(negedge clk);
        b_payload_data = a_payload_data;
        a_sequence     = 8'h20;
        b_sequence     = 8'h22;
        a_empty        = 1'b0;
        b_empty        = 1'b0;
        check_after_clock(
            1'b0, SINGLE_A, 1'b1, 1'b0, 1'b0, 6'b000001,
            1'b0, 1'b1, 1'b0, 1'b0, 8'h20
        );

        // B is older across the FF-to-00 wrap.
        apply_reset;
        @(negedge clk);
        a_sequence = 8'h00;
        b_sequence = 8'hFF;
        a_empty    = 1'b0;
        b_empty    = 1'b0;
        check_after_clock(
            1'b0, SINGLE_B, 1'b0, 1'b1, 1'b0, 6'b000001,
            1'b0, 1'b1, 1'b0, 1'b0, 8'hFF
        );

        // A distance of exactly 128 is ambiguous and consumes both heads.
        apply_reset;
        @(negedge clk);
        a_sequence = 8'h00;
        b_sequence = 8'h80;
        a_empty    = 1'b0;
        b_empty    = 1'b0;
        check_after_clock(
            1'b1, PAIR, 1'b1, 1'b1, 1'b0, 6'b000001,
            1'b0, 1'b0, 1'b1, 1'b0, 8'h00
        );

        // Runtime timeout setting: A-only result after three wait clocks.
        apply_reset;
        pair_timeout_cycles = 32'd3;
        @(negedge clk);
        a_sequence = 8'h30;
        a_seq_gap  = 1'b1;
        a_empty    = 1'b0;
        b_empty    = 1'b1;
        check_after_clock(
            1'b0, NONE, 1'b0, 1'b0, 1'b0, 6'b000000,
            1'b0, 1'b0, 1'b0, 1'b1, 8'h00
        );
        check_after_clock(
            1'b0, NONE, 1'b0, 1'b0, 1'b0, 6'b000000,
            1'b0, 1'b0, 1'b0, 1'b1, 8'h00
        );
        check_after_clock(
            1'b1, SINGLE_A, 1'b1, 1'b0, 1'b0, 6'b000000,
            1'b1, 1'b0, 1'b0, 1'b0, 8'h30
        );
        check_gap_metadata(1'b1, 1'b0, 1'b1);

        // A different runtime setting applies without changing the parameter.
        apply_reset;
        pair_timeout_cycles = 32'd2;
        @(negedge clk);
        a_empty    = 1'b1;
        b_empty    = 1'b0;
        b_sequence = 8'h40;
        b_seq_gap  = 1'b1;
        check_after_clock(
            1'b0, NONE, 1'b0, 1'b0, 1'b0, 6'b000000,
            1'b0, 1'b0, 1'b0, 1'b1, 8'h00
        );
        check_after_clock(
            1'b1, SINGLE_B, 1'b0, 1'b1, 1'b0, 6'b000000,
            1'b1, 1'b0, 1'b0, 1'b0, 8'h40
        );
        check_gap_metadata(1'b0, 1'b1, 1'b1);

        // clear is sampled synchronously and cancels an active wait.
        apply_reset;
        pair_timeout_cycles = 32'd3;
        @(negedge clk);
        a_empty = 1'b0;
        b_empty = 1'b1;
        check_after_clock(
            1'b0, NONE, 1'b0, 1'b0, 1'b0, 6'b000000,
            1'b0, 1'b0, 1'b0, 1'b1, 8'h00
        );
        @(negedge clk);
        clear = 1'b1;
        #1;
        if (pair_wait_active !== 1'b1) begin
            error_count = error_count + 1;
            $display("[FAIL] clear acted asynchronously at t=%0t", $time);
        end
        check_after_clock(
            1'b0, NONE, 1'b0, 1'b0, 1'b0, 6'b000000,
            1'b0, 1'b0, 1'b0, 1'b0, 8'h00
        );
        @(negedge clk);
        clear   = 1'b0;
        a_empty = 1'b1;

        if (error_count == 0)
            $display("[PASS] All pair_matcher final-interface tests passed.");
        else
            $display("[FAIL] pair_matcher tests: %0d error(s).",
                     error_count);

        $finish;
    end

    initial begin
        #100_000;
        $display("[FAIL] pair_matcher global timeout");
        $finish;
    end

endmodule
