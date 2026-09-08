`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// raw_frame_buffer Self-checking Testbench
//
// 검증 항목
//   1. Payload 0Byte Frame의 전체 Byte 순서
//   2. tx_ready=0일 때 tx_valid/tx_data 유지
//   3. Payload 3Byte 및 16Byte의 저장 순서
//   4. CRC High -> Low 순서
//   5. 내부 2-Frame FIFO와 Overflow Pulse/Count
//   6. 잘못된 LEN 차단
//////////////////////////////////////////////////////////////////////////////////

module tb_raw_frame_buffer;

    reg          clk;
    reg          reset_p;
    reg          clear;
    reg          statistics_clear;

    reg          in_valid;
    wire         in_ready;
    reg  [7:0]   in_frame_length;
    reg  [7:0]   in_device_id;
    reg  [7:0]   in_command;
    reg  [7:0]   in_sequence;
    reg  [127:0] in_payload_data;
    reg  [15:0]  in_received_crc;
    reg          in_seq_gap;

    wire         tx_valid;
    reg          tx_ready;
    wire [7:0]   tx_data;

    wire         buffer_busy;
    wire         buffer_full;
    wire [7:0]   buffered_count;
    wire         frame_done;
    wire         frame_done_seq_gap;
    wire         overflow_pulse;
    wire [15:0]  overflow_count;
    wire         length_error_pulse;

    integer error_count;
    integer expected_write;
    integer expected_read;
    integer frame_done_count;
    integer wait_count;

    reg [7:0] expected_bytes [0:255];
    reg [7:0] held_tx_data;
    reg       last_done_seq_gap;

    raw_frame_buffer #(
        .SYNC1       (8'hA5),
        .SYNC2       (8'h5A),
        .FRAME_DEPTH (2)
    ) dut (
        .clk                  (clk),
        .reset_p              (reset_p),
        .clear                (clear),
        .statistics_clear     (statistics_clear),

        .in_valid             (in_valid),
        .in_ready             (in_ready),
        .in_frame_length      (in_frame_length),
        .in_device_id         (in_device_id),
        .in_command           (in_command),
        .in_sequence          (in_sequence),
        .in_payload_data      (in_payload_data),
        .in_received_crc      (in_received_crc),
        .in_seq_gap           (in_seq_gap),

        .tx_valid             (tx_valid),
        .tx_ready             (tx_ready),
        .tx_data              (tx_data),

        .buffer_busy          (buffer_busy),
        .buffer_full          (buffer_full),
        .buffered_count       (buffered_count),
        .frame_done           (frame_done),
        .frame_done_seq_gap   (frame_done_seq_gap),
        .overflow_pulse       (overflow_pulse),
        .overflow_count       (overflow_count),
        .length_error_pulse   (length_error_pulse)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // tx_valid && tx_ready에서 실제 전달되는 Byte를 자동 검사한다.
    always @(posedge clk) begin
        if (!reset_p && tx_valid && tx_ready) begin
            if (expected_read >= expected_write) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] time=%0t unexpected TX byte=%h",
                    $time,
                    tx_data
                );
            end
            else if (tx_data !== expected_bytes[expected_read]) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] time=%0t byte[%0d]=%h expected=%h",
                    $time,
                    expected_read,
                    tx_data,
                    expected_bytes[expected_read]
                );
            end

            expected_read = expected_read + 1;
        end

        // frame_done과 함께 들어오는 Metadata를 저장한다.
        if (!reset_p && frame_done) begin
            frame_done_count  = frame_done_count + 1;
            last_done_seq_gap = frame_done_seq_gap;
        end
    end

    task apply_reset;
        begin
            reset_p = 1'b1;
            repeat (2) @(negedge clk);
            reset_p = 1'b0;
            @(negedge clk);
        end
    endtask

    // 정상 Frame의 예상 UART Byte 배열을 만든다.
    task append_expected_frame;
        input [7:0]   length;
        input [7:0]   device_id;
        input [7:0]   command;
        input [7:0]   sequence;
        input [127:0] payload;
        input [15:0]  crc;

        integer payload_length;
        integer payload_index;
        integer shift_amount;
        begin
            expected_bytes[expected_write] = 8'hA5;
            expected_write = expected_write + 1;

            expected_bytes[expected_write] = 8'h5A;
            expected_write = expected_write + 1;

            expected_bytes[expected_write] = length;
            expected_write = expected_write + 1;

            expected_bytes[expected_write] = device_id;
            expected_write = expected_write + 1;

            expected_bytes[expected_write] = command;
            expected_write = expected_write + 1;

            expected_bytes[expected_write] = sequence;
            expected_write = expected_write + 1;

            payload_length = length - 3;

            for (payload_index = 0;
                 payload_index < payload_length;
                 payload_index = payload_index + 1) begin

                shift_amount = (payload_length - 1 - payload_index) * 8;
                expected_bytes[expected_write] = payload >> shift_amount;
                expected_write = expected_write + 1;
            end

            expected_bytes[expected_write] = crc[15:8];
            expected_write = expected_write + 1;

            expected_bytes[expected_write] = crc[7:0];
            expected_write = expected_write + 1;
        end
    endtask

    // Frame 입력을 1클럭 Pulse로 넣고 Event 출력을 검사한다.
    task send_frame;
        input [7:0]   length;
        input [7:0]   device_id;
        input [7:0]   command;
        input [7:0]   sequence;
        input [127:0] payload;
        input [15:0]  crc;
        input         seq_gap;
        input         expected_overflow;
        input         expected_length_error;
        begin
            @(negedge clk);

            in_valid        = 1'b1;
            in_frame_length = length;
            in_device_id    = device_id;
            in_command      = command;
            in_sequence     = sequence;
            in_payload_data = payload;
            in_received_crc = crc;
            in_seq_gap      = seq_gap;

            @(posedge clk);
            #1;

            if ((overflow_pulse     !== expected_overflow) ||
                (length_error_pulse !== expected_length_error)) begin
                error_count = error_count + 1;
                $display(
                    "[FAIL] time=%0t overflow=%b/%b length_error=%b/%b",
                    $time,
                    overflow_pulse,
                    expected_overflow,
                    length_error_pulse,
                    expected_length_error
                );
            end

            @(negedge clk);
            in_valid = 1'b0;
        end
    endtask

    initial begin
        reset_p            = 1'b1;
        clear              = 1'b0;
        statistics_clear   = 1'b0;
        in_valid           = 1'b0;
        in_frame_length    = 8'd0;
        in_device_id       = 8'd0;
        in_command         = 8'd0;
        in_sequence        = 8'd0;
        in_payload_data    = 128'd0;
        in_received_crc    = 16'd0;
        in_seq_gap         = 1'b0;
        tx_ready           = 1'b0;

        error_count        = 0;
        expected_write     = 0;
        expected_read      = 0;
        frame_done_count   = 0;
        wait_count         = 0;
        held_tx_data       = 8'd0;
        last_done_seq_gap  = 1'b0;

        apply_reset;

        // ---------------------------------------------------------------------
        // 1. Payload 0Byte: A5 5A 03 01 10 20 BE EF
        // ---------------------------------------------------------------------
        append_expected_frame(
            8'd3, 8'h01, 8'h10, 8'h20, 128'd0, 16'hBEEF
        );

        send_frame(
            8'd3, 8'h01, 8'h10, 8'h20, 128'd0, 16'hBEEF,
            1'b0, 1'b0, 1'b0
        );

        // Serializer가 첫 Byte A5를 준비할 때까지 기다린다.
        wait (tx_valid === 1'b1);
        @(negedge clk);

        // ---------------------------------------------------------------------
        // 2. tx_ready=0이면 같은 Byte를 유지해야 한다.
        // ---------------------------------------------------------------------
        held_tx_data = tx_data;

        repeat (3) begin
            @(posedge clk);
            #1;

            if ((tx_valid !== 1'b1) || (tx_data !== held_tx_data)) begin
                error_count = error_count + 1;
                $display("[FAIL] time=%0t TX data changed during stall", $time);
            end
        end

        // ---------------------------------------------------------------------
        // 3. 첫 Frame이 대기 중일 때 Payload 3Byte Frame을 FIFO에 넣는다.
        //    Payload는 11 -> 22 -> 33 순서로 출력되어야 한다.
        // ---------------------------------------------------------------------
        append_expected_frame(
            8'd6, 8'h02, 8'h31, 8'h21,
            128'h00000000000000000000000000112233,
            16'hCAFE
        );

        send_frame(
            8'd6, 8'h02, 8'h31, 8'h21,
            128'h00000000000000000000000000112233,
            16'hCAFE,
            1'b0, 1'b0, 1'b0
        );

        // Payload 16Byte Frame도 FIFO에 저장한다.
        append_expected_frame(
            8'd19, 8'h03, 8'h32, 8'h22,
            128'h000102030405060708090A0B0C0D0E0F,
            16'h1234
        );

        send_frame(
            8'd19, 8'h03, 8'h32, 8'h22,
            128'h000102030405060708090A0B0C0D0E0F,
            16'h1234,
            1'b1, 1'b0, 1'b0
        );

        if ((buffer_full !== 1'b1) || (buffered_count !== 8'd2)) begin
            error_count = error_count + 1;
            $display(
                "[FAIL] time=%0t FIFO full/count mismatch full=%b count=%0d",
                $time,
                buffer_full,
                buffered_count
            );
        end

        // ---------------------------------------------------------------------
        // 4. 정상 Backpressure:
        //    FIFO가 Full이어도 Producer가 valid/data를 유지하면 유실이
        //    아니므로 overflow가 발생하지 않아야 한다. 공간이 생기면
        //    같은 요청을 그대로 Handshake한다.
        // ---------------------------------------------------------------------
        append_expected_frame(
            8'd3, 8'h04, 8'h33, 8'h23, 128'd0, 16'h5678
        );

        @(negedge clk);
        in_valid        = 1'b1;
        in_frame_length = 8'd3;
        in_device_id    = 8'h04;
        in_command      = 8'h33;
        in_sequence     = 8'h23;
        in_payload_data = 128'd0;
        in_received_crc = 16'h5678;
        in_seq_gap      = 1'b1;

        repeat (3) begin
            @(posedge clk);
            #1;

            if ((in_ready !== 1'b0) ||
                (overflow_pulse !== 1'b0) ||
                (overflow_count !== 16'd0)) begin

                error_count = error_count + 1;
                $display(
                    "[FAIL] time=%0t normal backpressure ready=%b pulse=%b count=%0d",
                    $time,
                    in_ready,
                    overflow_pulse,
                    overflow_count
                );
            end
        end

        @(negedge clk);
        tx_ready = 1'b1;

        wait (in_ready === 1'b1);
        @(posedge clk);
        #1;

        if ((overflow_pulse !== 1'b0) ||
            (overflow_count !== 16'd0)) begin

            error_count = error_count + 1;
            $display(
                "[FAIL] time=%0t accepted stalled request counted as overflow",
                $time
            );
        end

        @(negedge clk);
        in_valid = 1'b0;

        // UART가 저장된 4개 Frame을 모두 보낸다.

        wait_count = 0;
        while ((expected_read < expected_write) && (wait_count < 200)) begin
            @(negedge clk);
            wait_count = wait_count + 1;
        end

        if (expected_read !== expected_write) begin
            error_count = error_count + 1;
            $display(
                "[FAIL] TX byte count=%0d expected=%0d",
                expected_read,
                expected_write
            );
        end

        // 마지막 frame_done Pulse가 Monitor에 반영될 시간을 준다.
        repeat (2) @(posedge clk);

        if (frame_done_count !== 4) begin
            error_count = error_count + 1;
            $display(
                "[FAIL] frame_done_count=%0d expected=4",
                frame_done_count
            );
        end

        // 마지막 Frame에 들어 있던 seq_gap Metadata 확인
        if (last_done_seq_gap !== 1'b1) begin
            error_count = error_count + 1;
            $display("[FAIL] final frame seq_gap metadata mismatch");
        end

        // ---------------------------------------------------------------------
        // ---------------------------------------------------------------------
        // 5. 실제 유실:
        //    Full에서 막힌 요청을 Handshake 전에 철회하면 정확히 한 번
        //    overflow로 기록해야 한다.
        // ---------------------------------------------------------------------
        @(negedge clk);
        tx_ready = 1'b0;

        send_frame(
            8'd3, 8'h10, 8'h40, 8'h30, 128'd0, 16'h1001,
            1'b0, 1'b0, 1'b0
        );

        wait (tx_valid === 1'b1);

        send_frame(
            8'd3, 8'h11, 8'h41, 8'h31, 128'd0, 16'h1002,
            1'b0, 1'b0, 1'b0
        );

        send_frame(
            8'd3, 8'h12, 8'h42, 8'h32, 128'd0, 16'h1003,
            1'b0, 1'b0, 1'b0
        );

        if ((buffer_full !== 1'b1) || (in_ready !== 1'b0)) begin
            error_count = error_count + 1;
            $display("[FAIL] withdrawal setup did not fill FIFO");
        end

        @(negedge clk);
        in_valid        = 1'b1;
        in_frame_length = 8'd3;
        in_device_id    = 8'h13;
        in_command      = 8'h43;
        in_sequence     = 8'h33;
        in_payload_data = 128'd0;
        in_received_crc = 16'h1004;
        in_seq_gap      = 1'b0;

        repeat (2) begin
            @(posedge clk);
            #1;
            if ((overflow_pulse !== 1'b0) ||
                (overflow_count !== 16'd0)) begin

                error_count = error_count + 1;
                $display("[FAIL] held blocked request counted as overflow");
            end
        end

        @(negedge clk);
        in_valid = 1'b0;

        @(posedge clk);
        #1;
        if ((overflow_pulse !== 1'b1) ||
            (overflow_count !== 16'd1)) begin

            error_count = error_count + 1;
            $display(
                "[FAIL] withdrawn request overflow pulse=%b count=%0d expected=1/1",
                overflow_pulse,
                overflow_count
            );
        end

        @(posedge clk);
        #1;
        if ((overflow_pulse !== 1'b0) ||
            (overflow_count !== 16'd1)) begin

            error_count = error_count + 1;
            $display("[FAIL] withdrawal overflow was not exactly one pulse");
        end

        // statistics_clear는 FIFO 내용과 무관하게 Count만 동기식 Clear한다.
        @(negedge clk);
        statistics_clear = 1'b1;
        #1;
        if (overflow_count !== 16'd1) begin
            error_count = error_count + 1;
            $display("[FAIL] statistics_clear changed count asynchronously");
        end

        @(posedge clk);
        #1;
        if ((overflow_count !== 16'd0) || (buffer_full !== 1'b1)) begin
            error_count = error_count + 1;
            $display("[FAIL] statistics_clear result/state mismatch");
        end

        @(negedge clk);
        statistics_clear = 1'b0;
        clear = 1'b1;
        #1;
        if (buffer_full !== 1'b1) begin
            error_count = error_count + 1;
            $display("[FAIL] clear changed FIFO state asynchronously");
        end

        @(posedge clk);
        #1;
        if ((buffer_busy !== 1'b0) ||
            (buffered_count !== 8'd0)) begin

            error_count = error_count + 1;
            $display("[FAIL] synchronous clear did not empty buffer");
        end

        @(negedge clk);
        clear = 1'b0;

        // ---------------------------------------------------------------------
        // 6. LEN=2는 잘못된 Frame이므로 FIFO에 저장하지 않는다.
        // ---------------------------------------------------------------------
        send_frame(
            8'd2, 8'h05, 8'h34, 8'h24, 128'd0, 16'h9ABC,
            1'b0, 1'b0, 1'b1
        );

        repeat (2) @(posedge clk);

        if (buffer_busy !== 1'b0) begin
            error_count = error_count + 1;
            $display("[FAIL] invalid-length frame entered the buffer");
        end

        if (error_count == 0)
            $display("[PASS] All raw_frame_buffer core tests passed.");
        else
            $display("[FAIL] error_count=%0d", error_count);

        $finish;
    end

    // 무한 시뮬레이션 방지
    initial begin
        #10000;
        $display("[FAIL] Simulation timeout");
        $finish;
    end

endmodule
