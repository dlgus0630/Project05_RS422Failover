`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// tb_crc16_ccitt
//
// 검증 항목
//   1. Reset 후 초기값 16'hFFFF
//   2. 표준 검증 문자열 "123456789" -> 16'h29B1
//   3. 수신 CRC 불일치 검출
//   4. 실제 프로젝트 예시 프레임 범위 LEN~PAYLOAD 계산
//   5. Payload 0바이트와 Payload 16바이트 경계값
//   6. 중간에 버린 프레임 뒤 새 crc_start가 이전 계산을 초기화하는지 확인
//   7. crc_done/crc_ok/crc_error가 1클럭 펄스인지 확인
//////////////////////////////////////////////////////////////////////////////////

module tb_crc16_ccitt;

    localparam integer CLK_PERIOD_NS = 10;

    reg         clk;
    reg         reset_p;
    reg         clear;
    reg  [7:0]  crc_data;
    reg         crc_data_valid;
    reg         crc_start;
    reg         packet_valid;
    reg  [15:0] received_crc;

    wire [15:0] calculated_crc;
    wire        crc_done;
    wire        crc_ok;
    wire        crc_error;

    integer pass_count;
    integer fail_count;

    crc16_ccitt dut (
        .clk            (clk),
        .reset_p        (reset_p),
        .clear          (clear),
        .crc_data       (crc_data),
        .crc_data_valid (crc_data_valid),
        .crc_start      (crc_start),
        .packet_valid   (packet_valid),
        .received_crc   (received_crc),
        .calculated_crc (calculated_crc),
        .crc_done       (crc_done),
        .crc_ok         (crc_ok),
        .crc_error      (crc_error)
    );

    // 100MHz 클럭
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS / 2) clk = ~clk;
    end

    // CRC 모듈에 한 바이트를 전달한다.
    // first_byte=1이면 crc_start도 함께 올려 새 프레임 계산을 시작한다.
    // 이 task는 TB 반복 코드를 줄이기 위한 것으로 FPGA에 합성되지 않는다.
    task send_crc_byte;
        input [7:0] data;
        input       first_byte;
        begin
            @(negedge clk);
            crc_data       = data;
            crc_data_valid = 1'b1;
            crc_start      = first_byte;

            @(negedge clk);
            crc_data_valid = 1'b0;
            crc_start      = 1'b0;
        end
    endtask

    // packet_valid를 발생시킨 뒤 계산 CRC와 수신 CRC의 비교 결과를 검사한다.
    task check_crc_result;
        input [15:0] expected_calculated_crc;
        input [15:0] supplied_received_crc;
        input        expected_ok;
        reg          test_pass;
        begin
            test_pass = 1'b1;

            // frame_parser와 같이 received_crc를 먼저 안정시킨 상태에서
            // packet_valid를 1클럭 발생시킨다.
            @(negedge clk);
            received_crc = supplied_received_crc;
            packet_valid = 1'b1;

            @(negedge clk);
            packet_valid = 1'b0;
            #1;

            if (calculated_crc !== expected_calculated_crc) begin
                $display("[FAIL] calculated_crc: expected=%h, actual=%h",
                         expected_calculated_crc, calculated_crc);
                test_pass = 1'b0;
            end

            if (crc_done !== 1'b1) begin
                $display("[FAIL] crc_done이 비교 완료 시점에 1이 아님");
                test_pass = 1'b0;
            end

            if (expected_ok) begin
                if ((crc_ok !== 1'b1) || (crc_error !== 1'b0)) begin
                    $display("[FAIL] 정상 CRC 판정: crc_ok=%b, crc_error=%b",
                             crc_ok, crc_error);
                    test_pass = 1'b0;
                end
            end
            else begin
                if ((crc_ok !== 1'b0) || (crc_error !== 1'b1)) begin
                    $display("[FAIL] 오류 CRC 판정: crc_ok=%b, crc_error=%b",
                             crc_ok, crc_error);
                    test_pass = 1'b0;
                end
            end

            // 다음 클럭에는 결과 펄스가 모두 내려가야 한다.
            @(negedge clk);
            #1;

            if ((crc_done  !== 1'b0) ||
                (crc_ok    !== 1'b0) ||
                (crc_error !== 1'b0)) begin
                $display("[FAIL] 결과 신호가 1클럭 펄스가 아님");
                test_pass = 1'b0;
            end

            if (test_pass) begin
                pass_count = pass_count + 1;
                $display("[PASS] calculated=%h, received=%h, expected_ok=%b",
                         expected_calculated_crc,
                         supplied_received_crc,
                         expected_ok);
            end
            else begin
                fail_count = fail_count + 1;
            end
        end
    endtask

    initial begin
        reset_p        = 1'b1;
        clear          = 1'b0;
        crc_data       = 8'h00;
        crc_data_valid = 1'b0;
        crc_start      = 1'b0;
        packet_valid   = 1'b0;
        received_crc   = 16'h0000;
        pass_count     = 0;
        fail_count     = 0;

        repeat (5) @(negedge clk);

        // ---------------------------------------------------------------------
        // TEST 1: Reset
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Reset과 CRC 초기값");

        if ((calculated_crc === 16'hFFFF) &&
            (crc_done       === 1'b0)     &&
            (crc_ok         === 1'b0)     &&
            (crc_error      === 1'b0)) begin
            pass_count = pass_count + 1;
            $display("[PASS] Reset 출력 정상");
        end
        else begin
            fail_count = fail_count + 1;
            $display("[FAIL] Reset 출력 확인 필요");
        end

        reset_p = 1'b0;
        repeat (3) @(negedge clk);

        // ---------------------------------------------------------------------
        // TEST 2: CRC-16/CCITT-FALSE 표준 검증 벡터
        // ASCII "123456789"의 정답은 16'h29B1이다.
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 표준 벡터 123456789");
        send_crc_byte(8'h31, 1'b1); // '1': 새 CRC 시작
        send_crc_byte(8'h32, 1'b0); // '2'
        send_crc_byte(8'h33, 1'b0); // '3'
        send_crc_byte(8'h34, 1'b0); // '4'
        send_crc_byte(8'h35, 1'b0); // '5'
        send_crc_byte(8'h36, 1'b0); // '6'
        send_crc_byte(8'h37, 1'b0); // '7'
        send_crc_byte(8'h38, 1'b0); // '8'
        send_crc_byte(8'h39, 1'b0); // '9'
        check_crc_result(16'h29B1, 16'h29B1, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 3: 계산값은 같지만 수신 CRC의 LSB가 깨진 경우
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] CRC 불일치 검출");
        send_crc_byte(8'h31, 1'b1);
        send_crc_byte(8'h32, 1'b0);
        send_crc_byte(8'h33, 1'b0);
        send_crc_byte(8'h34, 1'b0);
        send_crc_byte(8'h35, 1'b0);
        send_crc_byte(8'h36, 1'b0);
        send_crc_byte(8'h37, 1'b0);
        send_crc_byte(8'h38, 1'b0);
        send_crc_byte(8'h39, 1'b0);
        check_crc_result(16'h29B1, 16'h29B0, 1'b0);

        // ---------------------------------------------------------------------
        // TEST 4: 프로젝트 예시
        // LEN=7, ID=01, CMD=02, SEQ=03, PAYLOAD=DE AD BE EF
        // CRC 계산 범위: 07 01 02 03 DE AD BE EF
        // Python binascii.crc_hqx(..., 16'hFFFF) 기준 정답: 16'hEF82
        // ---------------------------------------------------------------------
        $display("\n[TEST 4] 프로젝트 예시 Payload 4바이트");
        send_crc_byte(8'h07, 1'b1); // LEN
        send_crc_byte(8'h01, 1'b0); // DEVICE_ID
        send_crc_byte(8'h02, 1'b0); // CMD
        send_crc_byte(8'h03, 1'b0); // SEQ
        send_crc_byte(8'hDE, 1'b0); // PAYLOAD[0]
        send_crc_byte(8'hAD, 1'b0); // PAYLOAD[1]
        send_crc_byte(8'hBE, 1'b0); // PAYLOAD[2]
        send_crc_byte(8'hEF, 1'b0); // PAYLOAD[3]
        check_crc_result(16'hEF82, 16'hEF82, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 5: Payload 0바이트 경계값
        // 계산 범위: LEN=03, ID=01, CMD=10, SEQ=00
        // ---------------------------------------------------------------------
        $display("\n[TEST 5] Payload 0바이트");
        send_crc_byte(8'h03, 1'b1);
        send_crc_byte(8'h01, 1'b0);
        send_crc_byte(8'h10, 1'b0);
        send_crc_byte(8'h00, 1'b0);
        check_crc_result(16'h2B5F, 16'h2B5F, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 6: Payload 16바이트 최대 경계값
        // LEN=19(0x13), ID=2A, CMD=55, SEQ=FE, PAYLOAD=00~0F
        // ---------------------------------------------------------------------
        $display("\n[TEST 6] Payload 16바이트");
        send_crc_byte(8'h13, 1'b1);
        send_crc_byte(8'h2A, 1'b0);
        send_crc_byte(8'h55, 1'b0);
        send_crc_byte(8'hFE, 1'b0);
        send_crc_byte(8'h00, 1'b0);
        send_crc_byte(8'h01, 1'b0);
        send_crc_byte(8'h02, 1'b0);
        send_crc_byte(8'h03, 1'b0);
        send_crc_byte(8'h04, 1'b0);
        send_crc_byte(8'h05, 1'b0);
        send_crc_byte(8'h06, 1'b0);
        send_crc_byte(8'h07, 1'b0);
        send_crc_byte(8'h08, 1'b0);
        send_crc_byte(8'h09, 1'b0);
        send_crc_byte(8'h0A, 1'b0);
        send_crc_byte(8'h0B, 1'b0);
        send_crc_byte(8'h0C, 1'b0);
        send_crc_byte(8'h0D, 1'b0);
        send_crc_byte(8'h0E, 1'b0);
        send_crc_byte(8'h0F, 1'b0);
        check_crc_result(16'h883D, 16'h883D, 1'b1);

        // ---------------------------------------------------------------------
        // TEST 7: Parser가 프레임을 중간 폐기한 상황
        // 이전 계산이 남아 있어도 다음 LEN에서 crc_start=1이면 초기화되어야 한다.
        // ---------------------------------------------------------------------
        $display("\n[TEST 7] 중간 폐기 후 새 crc_start");
        send_crc_byte(8'h13, 1'b1); // 폐기될 프레임의 LEN
        send_crc_byte(8'hAA, 1'b0); // 일부 데이터
        send_crc_byte(8'hBB, 1'b0); // 일부 데이터

        // 새 정상 프레임 시작: 앞의 CRC 상태를 사용하면 안 된다.
        send_crc_byte(8'h03, 1'b1);
        send_crc_byte(8'h01, 1'b0);
        send_crc_byte(8'h10, 1'b0);
        send_crc_byte(8'h00, 1'b0);
        check_crc_result(16'h2B5F, 16'h2B5F, 1'b1);

        // ---------------------------------------------------------------------
        // 최종 결과
        // ---------------------------------------------------------------------
        $display("\n==================================================");
        $display("CRC16 TB RESULT: PASS=%0d, FAIL=%0d", pass_count, fail_count);

        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("TEST FAILED");

        $display("==================================================\n");
        $finish;
    end

    // 무한 시뮬레이션 방지
    initial begin
        #(100_000);
        $display("[FAIL] 전체 시뮬레이션 Timeout");
        $finish;
    end

endmodule
