`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// tb_frame_fifo
//
// frame_fifo의 핵심 동작만 파형과 자동 검사로 확인한다.
//
// 검증 순서
//   1. Reset 후 Empty
//   2. 프레임 1 Push
//   3. 프레임 2 Push 후 Full
//   4. Full 상태의 추가 Push 폐기와 Overflow Pulse
//   5. Full 상태에서 Pop과 Push 동시 수행
//   6. 저장 순서대로 Pop한 뒤 다시 Empty
//
// 불필요한 pass_count, fail_count, test_number는 사용하지 않는다.
//////////////////////////////////////////////////////////////////////////////////

module tb_frame_fifo;

    reg         clk;
    reg         reset_p;
    reg         clear;

    reg         push;
    reg  [7:0]  in_frame_length;
    reg  [7:0]  in_device_id;
    reg  [7:0]  in_command;
    reg  [7:0]  in_sequence;
    reg [127:0] in_payload_data;
    reg [15:0]  in_received_crc;
    reg         in_seq_gap;

    reg         pop;

    wire [7:0]   out_frame_length;
    wire [7:0]   out_device_id;
    wire [7:0]   out_command;
    wire [7:0]   out_sequence;
    wire [127:0] out_payload_data;
    wire [15:0]  out_received_crc;
    wire         out_seq_gap;

    wire       empty;
    wire       full;
    wire [1:0] count;
    wire       overflow_pulse;

    frame_fifo dut (
        .clk              (clk),
        .reset_p          (reset_p),
        .clear            (clear),
        .push             (push),
        .in_frame_length  (in_frame_length),
        .in_device_id     (in_device_id),
        .in_command       (in_command),
        .in_sequence      (in_sequence),
        .in_payload_data  (in_payload_data),
        .in_received_crc  (in_received_crc),
        .in_seq_gap       (in_seq_gap),
        .pop              (pop),
        .out_frame_length (out_frame_length),
        .out_device_id    (out_device_id),
        .out_command      (out_command),
        .out_sequence     (out_sequence),
        .out_payload_data (out_payload_data),
        .out_received_crc (out_received_crc),
        .out_seq_gap      (out_seq_gap),
        .empty            (empty),
        .full             (full),
        .count            (count),
        .overflow_pulse   (overflow_pulse)
    );

    // Basys3의 100MHz와 같은 10ns 주기 클럭
    initial clk = 1'b0;
    always #5 clk = ~clk;

    initial begin
        reset_p         = 1'b1;
        clear           = 1'b0;
        push            = 1'b0;
        pop             = 1'b0;
        in_frame_length = 8'd0;
        in_device_id    = 8'd0;
        in_command      = 8'd0;
        in_sequence     = 8'd0;
        in_payload_data = 128'd0;
        in_received_crc = 16'd0;
        in_seq_gap      = 1'b0;

        // ---------------------------------------------------------------------
        // 1. Reset 해제 후 FIFO는 비어 있어야 한다.
        // ---------------------------------------------------------------------
        repeat (2) @(negedge clk);
        reset_p = 1'b0;
        @(negedge clk);
        #1;

        if ((empty          !== 1'b1) ||
            (full           !== 1'b0) ||
            (count          !== 2'd0) ||
            (overflow_pulse !== 1'b0)) begin
            $display("[FAIL] Reset 상태가 올바르지 않음");
            $finish;
        end

        // ---------------------------------------------------------------------
        // 2. 프레임 1 Push
        //    LEN=5이므로 Payload는 DE AD 두 바이트이다.
        // ---------------------------------------------------------------------
        @(negedge clk);
        in_frame_length = 8'd5;
        in_device_id    = 8'h01;
        in_command      = 8'h10;
        in_sequence     = 8'd1;
        in_payload_data = 128'h0000000000000000000000000000DEAD;
        in_received_crc = 16'h1111;
        in_seq_gap      = 1'b0;
        push            = 1'b1;

        @(negedge clk);
        push = 1'b0;
        #1;

        if ((empty            !== 1'b0) ||
            (full             !== 1'b0) ||
            (count            !== 2'd1) ||
            (out_frame_length !== 8'd5) ||
            (out_device_id    !== 8'h01) ||
            (out_command      !== 8'h10) ||
            (out_sequence     !== 8'd1) ||
            (out_payload_data !== 128'h0000000000000000000000000000DEAD) ||
            (out_received_crc !== 16'h1111) ||
            (out_seq_gap      !== 1'b0)) begin
            $display("[FAIL] 첫 번째 프레임 Push 또는 Head 출력 오류");
            $finish;
        end

        // ---------------------------------------------------------------------
        // 3. Gap 메타데이터가 포함된 프레임 2 Push
        //    두 번째 Entry까지 차면 full=1이지만 Head는 프레임 1을 유지한다.
        // ---------------------------------------------------------------------
        @(negedge clk);
        in_frame_length = 8'd4;
        in_device_id    = 8'h01;
        in_command      = 8'h10;
        in_sequence     = 8'd4;
        in_payload_data = 128'h000000000000000000000000000000BE;
        in_received_crc = 16'h2222;
        in_seq_gap      = 1'b1;
        push            = 1'b1;

        @(negedge clk);
        push = 1'b0;
        #1;

        if ((full             !== 1'b1) ||
            (count            !== 2'd2) ||
            (out_sequence     !== 8'd1) ||
            (out_received_crc !== 16'h1111)) begin
            $display("[FAIL] 두 번째 프레임 Push 또는 FIFO 순서 오류");
            $finish;
        end

        // ---------------------------------------------------------------------
        // 4. Full 상태에서 Pop 없이 프레임 3 Push
        //    프레임 3은 저장되지 않고 overflow_pulse만 1클럭 발생한다.
        // ---------------------------------------------------------------------
        @(negedge clk);
        in_frame_length = 8'd3;
        in_device_id    = 8'h02;
        in_command      = 8'h20;
        in_sequence     = 8'd5;
        in_payload_data = 128'd0;
        in_received_crc = 16'h3333;
        in_seq_gap      = 1'b0;
        push            = 1'b1;

        @(negedge clk);
        push = 1'b0;
        #1;

        if ((overflow_pulse !== 1'b1) ||
            (count          !== 2'd2) ||
            (out_sequence   !== 8'd1)) begin
            $display("[FAIL] Full 상태의 Push 폐기 또는 Overflow 오류");
            $finish;
        end

        // 다음 클럭에는 Overflow Pulse가 다시 0이어야 한다.
        @(negedge clk);
        #1;

        if (overflow_pulse !== 1'b0) begin
            $display("[FAIL] overflow_pulse가 1클럭보다 길게 유지됨");
            $finish;
        end

        // ---------------------------------------------------------------------
        // 5. Full 상태에서 프레임 1 Pop과 프레임 3 Push를 동시에 수행
        //    프레임 수는 2를 유지하고 새 Head는 프레임 2가 된다.
        // ---------------------------------------------------------------------
        @(negedge clk);
        push = 1'b1;
        pop  = 1'b1;

        @(negedge clk);
        push = 1'b0;
        pop  = 1'b0;
        #1;

        if ((full             !== 1'b1) ||
            (count            !== 2'd2) ||
            (overflow_pulse   !== 1'b0) ||
            (out_frame_length !== 8'd4) ||
            (out_device_id    !== 8'h01) ||
            (out_command      !== 8'h10) ||
            (out_sequence     !== 8'd4) ||
            (out_payload_data !== 128'h000000000000000000000000000000BE) ||
            (out_received_crc !== 16'h2222) ||
            (out_seq_gap      !== 1'b1)) begin
            $display("[FAIL] 동시 Pop/Push 또는 프레임 2 Head 출력 오류");
            $finish;
        end

        // ---------------------------------------------------------------------
        // 6. 프레임 2 Pop
        //    다음 Head는 방금 저장한 프레임 3이어야 한다.
        // ---------------------------------------------------------------------
        @(negedge clk);
        pop = 1'b1;

        @(negedge clk);
        pop = 1'b0;
        #1;

        if ((empty            !== 1'b0) ||
            (full             !== 1'b0) ||
            (count            !== 2'd1) ||
            (out_frame_length !== 8'd3) ||
            (out_device_id    !== 8'h02) ||
            (out_command      !== 8'h20) ||
            (out_sequence     !== 8'd5) ||
            (out_payload_data !== 128'd0) ||
            (out_received_crc !== 16'h3333) ||
            (out_seq_gap      !== 1'b0)) begin
            $display("[FAIL] 프레임 2 Pop 또는 프레임 3 Head 출력 오류");
            $finish;
        end

        // 프레임 3까지 Pop하면 다시 Empty가 된다.
        @(negedge clk);
        pop = 1'b1;

        @(negedge clk);
        pop = 1'b0;
        #1;

        if ((empty            !== 1'b1) ||
            (full             !== 1'b0) ||
            (count            !== 2'd0) ||
            (out_frame_length !== 8'd0) ||
            (out_sequence     !== 8'd0) ||
            (out_payload_data !== 128'd0)) begin
            $display("[FAIL] 마지막 Pop 후 Empty 상태 오류");
            $finish;
        end

        $display("FRAME FIFO TB: ALL TESTS PASSED");
        $finish;
    end

    // 예상치 못한 정지로 무한 시뮬레이션이 되는 것을 방지한다.
    initial begin
        #1000;
        $display("[FAIL] 전체 시뮬레이션 Timeout");
        $finish;
    end

endmodule
