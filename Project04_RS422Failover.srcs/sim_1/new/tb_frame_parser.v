`timescale 1ns / 1ps

module tb_frame_parser;

    localparam integer CLK_PERIOD_NS = 10;

    reg         clk;
    reg         reset_p;
    reg         clear;
    reg  [7:0]  rx_data;
    reg         rx_valid;
    reg         rx_frame_error;

    wire [7:0]   frame_length;
    wire [7:0]   device_id;
    wire [7:0]   command;
    wire [7:0]   sequence;
    wire [127:0] payload_data;
    wire [15:0]  received_crc;
    wire         packet_valid;

    wire [7:0] crc_data;
    wire       crc_data_valid;
    wire       crc_start;

    wire length_error;
    wire interbyte_timeout;
    wire frame_timeout;

    // Timeout 펄스가 실제로 한 번이라도 나왔는지만 기억한다.
    // DUT 레지스터가 아니라 테스트벤치 관찰용 신호이다.
    reg interbyte_timeout_seen;
    reg frame_timeout_seen;
    integer error_count;

    frame_parser #(
        // 단위 테스트 시간을 줄이기 위한 값이다.
        // 실제 합성 기본값은 50,000 / 500,000클럭이다.
        .INTERBYTE_TIMEOUT_CLKS(20),
        .FRAME_TIMEOUT_CLKS    (100)
    ) dut (
        .clk               (clk),
        .reset_p           (reset_p),
        .clear             (clear),
        .rx_data           (rx_data),
        .rx_valid          (rx_valid),
        .rx_frame_error    (rx_frame_error),

        .frame_length      (frame_length),
        .device_id         (device_id),
        .command           (command),
        .sequence          (sequence),
        .payload_data      (payload_data),
        .received_crc      (received_crc),
        .packet_valid       (packet_valid),

        .crc_data          (crc_data),
        .crc_data_valid    (crc_data_valid),
        .crc_start         (crc_start),

        .length_error      (length_error),
        .interbyte_timeout (interbyte_timeout),
        .frame_timeout     (frame_timeout)
    );

    // 100MHz 클럭
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS / 2) clk = ~clk;
    end

    // rx_valid를 1클럭만 High로 만들어 Parser에 한 바이트를 전달한다.
    // 이 task는 테스트벤치의 반복 코드를 줄이기 위한 시뮬레이션 문법이며
    // FPGA에 합성되는 회로가 아니다.
    task send_byte;
        input [7:0] data;
        begin
            @(negedge clk);
            rx_data  = data;
            rx_valid = 1'b1;

            @(negedge clk);
            rx_valid = 1'b0;
        end
    endtask

    // Inter-byte Timeout은 1클럭 펄스이므로 발생 사실만 저장한다.
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            interbyte_timeout_seen <= 1'b0;
            frame_timeout_seen     <= 1'b0;
        end
        else begin
            if (interbyte_timeout)
                interbyte_timeout_seen <= 1'b1;

            if (frame_timeout)
                frame_timeout_seen <= 1'b1;
        end
    end

    initial begin
        reset_p                = 1'b1;
        clear                  = 1'b0;
        rx_data                = 8'h00;
        rx_valid               = 1'b0;
        rx_frame_error         = 1'b0;
        interbyte_timeout_seen = 1'b0;
        frame_timeout_seen     = 1'b0;
        error_count            = 0;

        repeat (5) @(negedge clk);
        reset_p = 1'b0;
        repeat (3) @(negedge clk);

        // ---------------------------------------------------------------------
        // TEST 1: Payload 2바이트 정상 프레임
        // A5 5A | LEN=5 | DEV=11 CMD=22 SEQ=07 | DE AD | CRC=12 34
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Payload 2Byte 정상 프레임");
        send_byte(8'hA5);
        send_byte(8'h5A);
        send_byte(8'h05);
        send_byte(8'h11);
        send_byte(8'h22);
        send_byte(8'h07);
        send_byte(8'hDE);
        send_byte(8'hAD);
        send_byte(8'h12);
        send_byte(8'h34);

        if ((packet_valid      == 1'b1)       &&
            (frame_length      == 8'h05)       &&
            (device_id         == 8'h11)       &&
            (command           == 8'h22)       &&
            (sequence          == 8'h07)       &&
            (payload_data      == 128'h0000000000000000000000000000DEAD) &&
            (received_crc      == 16'h1234))
            $display("[PASS] 필드 조립과 packet_valid 정상");
        else begin
            error_count = error_count + 1;
            $display("[FAIL] 정상 프레임 조립 결과 확인 필요");
        end

        // packet_valid가 한 클럭 펄스인지 확인
        @(negedge clk);
        if (packet_valid == 1'b0)
            $display("[PASS] packet_valid 1클럭 펄스 정상");
        else begin
            error_count = error_count + 1;
            $display("[FAIL] packet_valid가 1클럭보다 김");
        end

        // ---------------------------------------------------------------------
        // TEST 2: 잘못된 LEN 폐기 후 A5 A5 5A 재동기화
        // LEN=2는 허용 범위 3~19 밖이므로 length_error가 발생해야 한다.
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Length Error와 재동기화");
        send_byte(8'hA5);
        send_byte(8'h5A);
        send_byte(8'h02);

        if ((length_error == 1'b1) && (packet_valid == 1'b0))
            $display("[PASS] LEN=2 검출 후 프레임 폐기");
        else begin
            error_count = error_count + 1;
            $display("[FAIL] Length Error 처리 확인 필요");
        end

        // A5 A5 5A에서 두 번째 A5를 새 SYNC1으로 사용한다.
        send_byte(8'hA5);
        send_byte(8'hA5);
        send_byte(8'h5A);
        send_byte(8'h03); // Payload 없음
        send_byte(8'h44); // DEVICE_ID
        send_byte(8'h55); // CMD
        send_byte(8'h66); // SEQ
        send_byte(8'hBE); // CRC High
        send_byte(8'hEF); // CRC Low

        if ((packet_valid == 1'b1)  &&
            (frame_length == 8'h03)  &&
            (device_id    == 8'h44)  &&
            (command      == 8'h55)  &&
            (sequence     == 8'h66)  &&
            (payload_data == 128'd0) &&
            (received_crc == 16'hBEEF))
            $display("[PASS] A5 A5 5A 재동기화와 0Byte Payload 정상");
        else begin
            error_count = error_count + 1;
            $display("[FAIL] 재동기화 결과 확인 필요");
        end

        @(negedge clk);

        // ---------------------------------------------------------------------
        // TEST 3: 프레임 중간 UART Framing Error
        // Parser는 조립 중인 프레임 전체를 버리고 WAIT_SYNC1로 돌아가야 한다.
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] UART Framing Error 시 진행 프레임 폐기");
        send_byte(8'hA5);
        send_byte(8'h5A);
        send_byte(8'h04);
        send_byte(8'h21);

        @(negedge clk);
        rx_frame_error = 1'b1;
        @(negedge clk);
        rx_frame_error = 1'b0;

        // 오류 전 프레임의 나머지처럼 보이는 바이트는 모두 무시되어야 한다.
        send_byte(8'h31);
        send_byte(8'h41);
        send_byte(8'h51);
        send_byte(8'h61);
        send_byte(8'h71);

        if ((packet_valid == 1'b0) && (dut.state == 4'd0))
            $display("[PASS] 진행 중 프레임 폐기 후 WAIT_SYNC1 복귀");
        else begin
            error_count = error_count + 1;
            $display("[FAIL] UART Framing Error 복귀 상태 확인 필요");
        end

        // ---------------------------------------------------------------------
        // TEST 4: Inter-byte Timeout
        // ---------------------------------------------------------------------
        $display("\n[TEST 4] Inter-byte Timeout");
        send_byte(8'hA5);
        send_byte(8'h5A);
        send_byte(8'h03);

        // 다음 바이트를 보내지 않고 설정값 20클럭보다 길게 기다린다.
        repeat (25) @(negedge clk);

        if ((interbyte_timeout_seen == 1'b1) && (dut.state == 4'd0))
            $display("[PASS] Inter-byte Timeout 후 WAIT_SYNC1 복귀");
        else begin
            error_count = error_count + 1;
            $display("[FAIL] Inter-byte Timeout 처리 확인 필요");
        end

        // Timeout 후에도 새 정상 프레임을 받을 수 있는지 확인한다.
        send_byte(8'hA5);
        send_byte(8'h5A);
        send_byte(8'h03);
        send_byte(8'hAA);
        send_byte(8'h10);
        send_byte(8'h01);
        send_byte(8'hCA);
        send_byte(8'hFE);

        if ((packet_valid == 1'b1) &&
            (device_id    == 8'hAA) &&
            (command      == 8'h10) &&
            (sequence     == 8'h01) &&
            (received_crc == 16'hCAFE))
            $display("[PASS] Timeout 이후 정상 프레임 수신 복구");
        else begin
            error_count = error_count + 1;
            $display("[FAIL] Timeout 이후 복구 확인 필요");
        end

        @(negedge clk);

        // ---------------------------------------------------------------------
        // TEST 5: Frame Timeout
        // 바이트 사이는 500us 이내이지만 전체 프레임 시간이 5ms를 넘는 상황이다.
        // TB에서는 각각 20클럭 / 100클럭으로 축소해서 같은 조건을 만든다.
        // ---------------------------------------------------------------------
        $display("\n[TEST 5] Frame Timeout");
        send_byte(8'hA5);
        send_byte(8'h5A);

        repeat (15) @(negedge clk);
        send_byte(8'h13); // LEN=19: Payload 16Byte

        repeat (15) @(negedge clk);
        send_byte(8'h01); // DEVICE_ID

        repeat (15) @(negedge clk);
        send_byte(8'h02); // CMD

        repeat (15) @(negedge clk);
        send_byte(8'h03); // SEQ

        repeat (15) @(negedge clk);
        send_byte(8'h11); // Payload 일부

        // 바이트 간격은 20클럭 미만으로 유지하면서 전체 100클럭을 넘긴다.
        repeat (15) @(negedge clk);
        send_byte(8'h22);
        repeat (15) @(negedge clk);

        if ((frame_timeout_seen == 1'b1) && (dut.state == 4'd0))
            $display("[PASS] Frame Timeout 후 WAIT_SYNC1 복귀");
        else begin
            error_count = error_count + 1;
            $display("[FAIL] Frame Timeout 처리 확인 필요");
        end

        if (error_count == 0)
            $display("[PASS] All frame_parser core tests passed.");
        else
            $display("[FAIL] frame_parser tests failed: %0d error(s).",
                     error_count);

        $finish;
    end

    // 무한 시뮬레이션 방지
    initial begin
        #(100_000);
        $display("[FAIL] 전체 시뮬레이션 Timeout");
        $finish;
    end

endmodule
