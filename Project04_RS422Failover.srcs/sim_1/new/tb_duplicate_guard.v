`timescale 1ns / 1ps

// duplicate_guard 핵심 정책 확인용 Self-checking Testbench
//
// 검증 항목
//   1. 처음 들어온 채택 프레임은 정상 출력
//   2. 그사이 다른 프레임이 있어도 최근 이력의 중복 프레임은 차단
//   3. decision_accept=0인 폐기 프레임은 출력 이력에 저장하지 않음
//   4. SEQUENCE가 같아도 DEVICE_ID가 다르면 별도 프레임으로 출력
//   5. HISTORY_DEPTH를 벗어난 오래된 이력은 제거

module tb_duplicate_guard;

    reg          clk;
    reg          reset_p;
    reg          clear;

    reg          decision_valid;
    reg          decision_accept;
    reg          statistics_clear;

    reg  [7:0]   in_frame_length;
    reg  [7:0]   in_device_id;
    reg  [7:0]   in_command;
    reg  [7:0]   in_sequence;
    reg  [127:0] in_payload_data;
    reg  [15:0]  in_received_crc;
    reg          in_seq_gap;
    reg          in_selected_b;

    wire         out_valid;
    wire [7:0]   out_frame_length;
    wire [7:0]   out_device_id;
    wire [7:0]   out_command;
    wire [7:0]   out_sequence;
    wire [127:0] out_payload_data;
    wire [15:0]  out_received_crc;
    wire         out_seq_gap;
    wire         out_selected_b;

    wire         duplicate_drop;
    wire [15:0]  duplicate_count;

    integer error_count;

    duplicate_guard #(
        .HISTORY_DEPTH (4)
    ) dut (
        .clk                (clk),
        .reset_p            (reset_p),
        .clear              (clear),

        .decision_valid     (decision_valid),
        .decision_accept    (decision_accept),
        .statistics_clear   (statistics_clear),

        .in_frame_length    (in_frame_length),
        .in_device_id       (in_device_id),
        .in_command         (in_command),
        .in_sequence        (in_sequence),
        .in_payload_data    (in_payload_data),
        .in_received_crc    (in_received_crc),
        .in_seq_gap         (in_seq_gap),
        .in_selected_b      (in_selected_b),

        .out_valid          (out_valid),
        .out_frame_length   (out_frame_length),
        .out_device_id      (out_device_id),
        .out_command        (out_command),
        .out_sequence       (out_sequence),
        .out_payload_data   (out_payload_data),
        .out_received_crc   (out_received_crc),
        .out_seq_gap        (out_seq_gap),
        .out_selected_b     (out_selected_b),

        .duplicate_drop     (duplicate_drop),
        .duplicate_count    (duplicate_count)
    );

    initial clk = 1'b0;
    always #5 clk = ~clk;

    task apply_reset;
        begin
            clear   = 1'b0;
            reset_p = 1'b1;
            repeat (2) @(negedge clk);
            reset_p = 1'b0;
            @(negedge clk);
        end
    endtask

    // 판정 결과를 1클럭 동안 입력하고 DUT 출력이 갱신된 뒤 검사한다.
    task send_and_check;
        input       accept;
        input [7:0] device_id;
        input [7:0] sequence;
        input       expected_out_valid;
        input       expected_duplicate_drop;
        input [15:0] expected_duplicate_count;
        begin
            @(negedge clk);

            decision_valid  = 1'b1;
            decision_accept = accept;
            in_frame_length = 8'd7;
            in_device_id    = device_id;
            in_command      = 8'h31;
            in_sequence     = sequence;
            in_payload_data = {
                96'h0,
                device_id,
                sequence,
                16'hA55A
            };
            in_received_crc = 16'h5AA5;
            in_seq_gap      = 1'b0;
            in_selected_b   = sequence[0];

            @(posedge clk);
            #1;

            if ((out_valid       !== expected_out_valid)       ||
                (duplicate_drop  !== expected_duplicate_drop)  ||
                (duplicate_count !== expected_duplicate_count)) begin

                error_count = error_count + 1;

                $display(
                    "[FAIL] time=%0t dev=%h seq=%h accept=%b out=%b/%b drop=%b/%b count=%0d/%0d",
                    $time,
                    device_id,
                    sequence,
                    accept,
                    out_valid,
                    expected_out_valid,
                    duplicate_drop,
                    expected_duplicate_drop,
                    duplicate_count,
                    expected_duplicate_count
                );
            end

            if (expected_out_valid || expected_duplicate_drop) begin
                if ((out_frame_length !== 8'd7)                  ||
                    (out_device_id    !== device_id)             ||
                    (out_command      !== 8'h31)                 ||
                    (out_sequence     !== sequence)              ||
                    (out_payload_data !== {
                        96'h0, device_id, sequence, 16'hA55A
                    })                                           ||
                    (out_received_crc !== 16'h5AA5)              ||
                    (out_seq_gap      !== 1'b0)                  ||
                    (out_selected_b   !== sequence[0])) begin

                    error_count = error_count + 1;
                    $display(
                        "[FAIL] time=%0t forwarded/drop metadata mismatch",
                        $time
                    );
                end
            end

            @(negedge clk);
            decision_valid  = 1'b0;
            decision_accept = 1'b0;

            // Pulse가 다음 클럭에 내려가는지도 확인한다.
            @(posedge clk);
            #1;

            if ((out_valid !== 1'b0) || (duplicate_drop !== 1'b0)) begin
                error_count = error_count + 1;
                $display("[FAIL] time=%0t output pulse is longer than one clock", $time);
            end
        end
    endtask

    initial begin
        reset_p          = 1'b1;
        clear            = 1'b0;
        decision_valid   = 1'b0;
        decision_accept  = 1'b0;
        statistics_clear = 1'b0;
        in_frame_length  = 8'd0;
        in_device_id     = 8'd0;
        in_command       = 8'd0;
        in_sequence      = 8'd0;
        in_payload_data  = 128'd0;
        in_received_crc  = 16'd0;
        in_seq_gap       = 1'b0;
        in_selected_b    = 1'b0;
        error_count      = 0;

        apply_reset;

        // ---------------------------------------------------------------------
        // 1. 처음 보는 프레임은 정상 출력한다.
        // ---------------------------------------------------------------------
        send_and_check(1'b1, 8'h01, 8'h10, 1'b1, 1'b0, 16'd0);

        // 다른 프레임도 정상 출력한다.
        send_and_check(1'b1, 8'h01, 8'h11, 1'b1, 1'b0, 16'd0);
        send_and_check(1'b1, 8'h01, 8'h12, 1'b1, 1'b0, 16'd0);

        // ---------------------------------------------------------------------
        // 2. 직전 프레임은 아니지만 최근 이력에 있는 SEQ=10은 차단한다.
        // ---------------------------------------------------------------------
        send_and_check(1'b1, 8'h01, 8'h10, 1'b0, 1'b1, 16'd1);

        // ---------------------------------------------------------------------
        // 3. decision_unit에서 폐기된 프레임은 출력도, 이력 저장도 하지 않는다.
        // ---------------------------------------------------------------------
        send_and_check(1'b0, 8'h01, 8'h20, 1'b0, 1'b0, 16'd1);

        // 같은 프레임이 나중에 채택되면 처음 출력되는 프레임이므로 통과한다.
        send_and_check(1'b1, 8'h01, 8'h20, 1'b1, 1'b0, 16'd1);

        // ---------------------------------------------------------------------
        // 4. SEQUENCE가 같아도 DEVICE_ID가 다르면 별도 프레임이다.
        // ---------------------------------------------------------------------
        send_and_check(1'b1, 8'h02, 8'h20, 1'b1, 1'b0, 16'd1);

        // ---------------------------------------------------------------------
        // 5. 깊이 4를 넘긴 오래된 이력이 제거되는지 확인한다.
        // ---------------------------------------------------------------------
        apply_reset;

        send_and_check(1'b1, 8'h01, 8'h01, 1'b1, 1'b0, 16'd0);
        send_and_check(1'b1, 8'h01, 8'h02, 1'b1, 1'b0, 16'd0);
        send_and_check(1'b1, 8'h01, 8'h03, 1'b1, 1'b0, 16'd0);
        send_and_check(1'b1, 8'h01, 8'h04, 1'b1, 1'b0, 16'd0);
        send_and_check(1'b1, 8'h01, 8'h05, 1'b1, 1'b0, 16'd0);

        // SEQ=01은 최근 4개 이력에서 밀려났으므로 다시 통과한다.
        send_and_check(1'b1, 8'h01, 8'h01, 1'b1, 1'b0, 16'd0);

        // ---------------------------------------------------------------------
        // 6. statistics_clear는 카운터만 동기적으로 지우고 이력은 보존한다.
        // ---------------------------------------------------------------------
        apply_reset;
        send_and_check(1'b1, 8'h03, 8'h71, 1'b1, 1'b0, 16'd0);
        send_and_check(1'b1, 8'h03, 8'h71, 1'b0, 1'b1, 16'd1);

        @(negedge clk);
        statistics_clear = 1'b1;
        #1;
        if (duplicate_count !== 16'd1) begin
            error_count = error_count + 1;
            $display("[FAIL] statistics_clear acted asynchronously at time=%0t",
                     $time);
        end

        @(posedge clk);
        #1;
        if (duplicate_count !== 16'd0) begin
            error_count = error_count + 1;
            $display("[FAIL] statistics_clear did not clear the counter");
        end

        @(negedge clk);
        statistics_clear = 1'b0;
        send_and_check(1'b1, 8'h03, 8'h71, 1'b0, 1'b1, 16'd1);

        // ---------------------------------------------------------------------
        // 7. clear is synchronous and clears both counter and duplicate history.
        // ---------------------------------------------------------------------
        @(negedge clk);
        clear = 1'b1;
        #1;
        if (duplicate_count !== 16'd1) begin
            error_count = error_count + 1;
            $display("[FAIL] clear acted asynchronously at time=%0t", $time);
        end

        @(posedge clk);
        #1;
        if ((duplicate_count !== 16'd0) ||
            (out_valid !== 1'b0) ||
            (duplicate_drop !== 1'b0)) begin
            error_count = error_count + 1;
            $display("[FAIL] synchronous clear did not reset duplicate_guard");
        end

        @(negedge clk);
        clear = 1'b0;
        send_and_check(1'b1, 8'h03, 8'h71, 1'b1, 1'b0, 16'd0);

        if (error_count == 0)
            $display("[PASS] All duplicate_guard final-interface tests passed.");
        else
            $display("[FAIL] error_count=%0d", error_count);

        $finish;
    end

    // 무한 시뮬레이션 방지
    initial begin
        #5000;
        $display("[FAIL] Simulation timeout");
        $finish;
    end

endmodule
