`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// frame_parser
//
// uart_rx가 복원한 1바이트(rx_data)를 rx_valid 펄스마다 받아
// 사용자 정의 프레임의 각 필드로 조립한다.
//
// 프레임 형식
//   SYNC1 | SYNC2 | LEN | DEVICE_ID | CMD | SEQ | PAYLOAD | CRC_H | CRC_L
//    A5      5A     3~19      1B        1B    1B     0~16B      1B      1B
//
// LEN 정의
//   LEN = DEVICE_ID(1) + CMD(1) + SEQ(1) + PAYLOAD 길이
//       = 3 + PAYLOAD 길이
//
// 주요 동작
//   1. 0xA5, 0x5A를 차례대로 찾는다.
//   2. LEN이 3~19인지 검사한다.
//   3. 헤더와 Payload를 순서대로 저장한다.
//   4. CRC High/Low를 저장한 뒤 packet_valid를 1클럭 출력한다.
//   5. UART Framing Error, Length Error, Timeout이면 현재 프레임을 버린다.
//
// CRC 경계
//   이 모듈은 수신 CRC를 저장하지만 CRC의 정답 여부는 판정하지 않는다.
//   LEN부터 마지막 PAYLOAD까지 crc_data/crc_data_valid로 내보내므로
//   뒤쪽 crc16_ccitt 모듈이 수신 중 CRC를 누적 계산할 수 있다.
//////////////////////////////////////////////////////////////////////////////////

module frame_parser #(
    parameter [7:0] SYNC1 = 8'hA5,
    parameter [7:0] SYNC2 = 8'h5A,

    // Basys3 100MHz 기준: 500us = 50,000클럭, 5ms = 500,000클럭
    // 테스트벤치에서는 시뮬레이션 시간을 줄이기 위해 작은 값으로 바꾼다.
    parameter integer INTERBYTE_TIMEOUT_CLKS = 50_000,
    parameter integer FRAME_TIMEOUT_CLKS     = 500_000
)(
    input  wire         clk,
    input  wire         reset_p,
    input  wire         clear,

    // uart_rx와 연결되는 입력
    input  wire [7:0]   rx_data,
    input  wire         rx_valid,
    input  wire         rx_frame_error,

    // 완성된 프레임의 필드
    output reg  [7:0]   frame_length,
    output reg  [7:0]   device_id,
    output reg  [7:0]   command,
    output reg  [7:0]   sequence,
    output reg  [127:0] payload_data,
    output reg  [15:0]  received_crc,
    output reg          packet_valid,

    // crc16_ccitt 모듈로 전달할 바이트 스트림
    output reg  [7:0]   crc_data,
    output reg          crc_data_valid,
    output reg          crc_start,

    // Parser 자체에서 검출하는 오류 펄스
    output reg          length_error,
    output reg          interbyte_timeout,
    output reg          frame_timeout
);

    // -------------------------------------------------------------------------
    // 프레임 Parser FSM
    // -------------------------------------------------------------------------
    localparam [3:0] ST_WAIT_SYNC1   = 4'd0;
    localparam [3:0] ST_WAIT_SYNC2   = 4'd1;
    localparam [3:0] ST_READ_LENGTH  = 4'd2;
    localparam [3:0] ST_READ_DEVICE  = 4'd3;
    localparam [3:0] ST_READ_COMMAND = 4'd4;
    localparam [3:0] ST_READ_SEQ     = 4'd5;
    localparam [3:0] ST_READ_PAYLOAD = 4'd6;
    localparam [3:0] ST_READ_CRC_H   = 4'd7;
    localparam [3:0] ST_READ_CRC_L   = 4'd8;

    reg [3:0] state;

    // 아직 받아야 하는 Payload 바이트 수: 0~16
    reg [4:0] payload_remaining;

    localparam integer INTERBYTE_CNT_WIDTH =
        (INTERBYTE_TIMEOUT_CLKS <= 2) ? 1 : $clog2(INTERBYTE_TIMEOUT_CLKS);

    localparam integer FRAME_CNT_WIDTH =
        (FRAME_TIMEOUT_CLKS <= 2) ? 1 : $clog2(FRAME_TIMEOUT_CLKS);

    reg [INTERBYTE_CNT_WIDTH-1:0] interbyte_count;
    reg [FRAME_CNT_WIDTH-1:0]     frame_count;

    // WAIT_SYNC1 이외의 상태는 프레임을 찾았거나 조립 중인 상태이다.
    wire parser_active;
    assign parser_active = (state != ST_WAIT_SYNC1);

    // -------------------------------------------------------------------------
    // Parser FSM, 필드 저장, Timeout 처리
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            state              <= ST_WAIT_SYNC1;
            payload_remaining  <= 5'd0;
            interbyte_count    <= {INTERBYTE_CNT_WIDTH{1'b0}};
            frame_count        <= {FRAME_CNT_WIDTH{1'b0}};

            frame_length       <= 8'd0;
            device_id          <= 8'd0;
            command            <= 8'd0;
            sequence           <= 8'd0;
            payload_data       <= 128'd0;
            received_crc       <= 16'd0;

            packet_valid       <= 1'b0;
            crc_data           <= 8'd0;
            crc_data_valid     <= 1'b0;
            crc_start          <= 1'b0;
            length_error       <= 1'b0;
            interbyte_timeout  <= 1'b0;
            frame_timeout      <= 1'b0;
        end
        else if (clear) begin
            state              <= ST_WAIT_SYNC1;
            payload_remaining  <= 5'd0;
            interbyte_count    <= {INTERBYTE_CNT_WIDTH{1'b0}};
            frame_count        <= {FRAME_CNT_WIDTH{1'b0}};

            frame_length       <= 8'd0;
            device_id          <= 8'd0;
            command            <= 8'd0;
            sequence           <= 8'd0;
            payload_data       <= 128'd0;
            received_crc       <= 16'd0;

            packet_valid       <= 1'b0;
            crc_data           <= 8'd0;
            crc_data_valid     <= 1'b0;
            crc_start          <= 1'b0;
            length_error       <= 1'b0;
            interbyte_timeout  <= 1'b0;
            frame_timeout      <= 1'b0;
        end
        else begin
            // 아래 신호들은 사건을 알리는 1클럭 펄스이다.
            packet_valid      <= 1'b0;
            crc_data_valid    <= 1'b0;
            crc_start         <= 1'b0;
            length_error      <= 1'b0;
            interbyte_timeout <= 1'b0;
            frame_timeout     <= 1'b0;

            // UART 바이트 자체가 무효이면 진행 중인 프레임 전체를 버린다.
            if (rx_frame_error) begin
                state             <= ST_WAIT_SYNC1;
                payload_remaining <= 5'd0;
                interbyte_count   <= {INTERBYTE_CNT_WIDTH{1'b0}};
                frame_count       <= {FRAME_CNT_WIDTH{1'b0}};
            end

            // SYNC1 이후 전체 프레임 수신 시간이 5ms를 넘은 경우
            else if (parser_active &&
                     (frame_count == FRAME_TIMEOUT_CLKS - 1)) begin
                state             <= ST_WAIT_SYNC1;
                payload_remaining <= 5'd0;
                interbyte_count   <= {INTERBYTE_CNT_WIDTH{1'b0}};
                frame_count       <= {FRAME_CNT_WIDTH{1'b0}};
                frame_timeout     <= 1'b1;
            end

            // 프레임 조립 중 다음 정상 바이트가 500us 동안 오지 않은 경우
            else if (parser_active && !rx_valid &&
                     (interbyte_count == INTERBYTE_TIMEOUT_CLKS - 1)) begin
                state             <= ST_WAIT_SYNC1;
                payload_remaining <= 5'd0;
                interbyte_count   <= {INTERBYTE_CNT_WIDTH{1'b0}};
                frame_count       <= {FRAME_CNT_WIDTH{1'b0}};
                interbyte_timeout <= 1'b1;
            end

            else begin
                // -------------------------------------------------------------
                // Timeout 카운터
                // -------------------------------------------------------------
                if (!parser_active) begin
                    interbyte_count <= {INTERBYTE_CNT_WIDTH{1'b0}};
                    frame_count     <= {FRAME_CNT_WIDTH{1'b0}};
                end
                else begin
                    frame_count <= frame_count + 1'b1;

                    if (rx_valid)
                        interbyte_count <= {INTERBYTE_CNT_WIDTH{1'b0}};
                    else
                        interbyte_count <= interbyte_count + 1'b1;
                end

                // Parser는 rx_valid가 1일 때만 rx_data를 해석한다.
                if (rx_valid) begin
                    case (state)
                        // -----------------------------------------------------
                        // 1. 첫 번째 SYNC 0xA5 탐색
                        // -----------------------------------------------------
                        ST_WAIT_SYNC1: begin
                            if (rx_data == SYNC1) begin
                                state           <= ST_WAIT_SYNC2;
                                frame_count     <= {FRAME_CNT_WIDTH{1'b0}};
                                interbyte_count <= {INTERBYTE_CNT_WIDTH{1'b0}};
                            end
                        end

                        // -----------------------------------------------------
                        // 2. 두 번째 SYNC 0x5A 확인
                        // -----------------------------------------------------
                        ST_WAIT_SYNC2: begin
                            if (rx_data == SYNC2) begin
                                state <= ST_READ_LENGTH;
                            end
                            else if (rx_data == SYNC1) begin
                                // A5 A5 5A처럼 들어온 경우 두 번째 A5를
                                // 새로운 SYNC1으로 인정해 빠르게 재동기화한다.
                                state       <= ST_WAIT_SYNC2;
                                frame_count <= {FRAME_CNT_WIDTH{1'b0}};
                            end
                            else begin
                                state <= ST_WAIT_SYNC1;
                            end
                        end

                        // -----------------------------------------------------
                        // 3. LEN 검사: 3~19만 허용
                        // -----------------------------------------------------
                        ST_READ_LENGTH: begin
                            if ((rx_data >= 8'd3) && (rx_data <= 8'd19)) begin
                                frame_length      <= rx_data;
                                payload_remaining <= rx_data - 8'd3;
                                payload_data      <= 128'd0;
                                received_crc      <= 16'd0;

                                // CRC 계산은 LEN부터 시작한다.
                                crc_data       <= rx_data;
                                crc_data_valid <= 1'b1;
                                crc_start      <= 1'b1;

                                state <= ST_READ_DEVICE;
                            end
                            else begin
                                // 잘못된 LEN이면 현재 프레임을 폐기한다.
                                state             <= ST_WAIT_SYNC1;
                                payload_remaining <= 5'd0;
                                interbyte_count   <= {INTERBYTE_CNT_WIDTH{1'b0}};
                                frame_count       <= {FRAME_CNT_WIDTH{1'b0}};
                                length_error      <= 1'b1;
                            end
                        end

                        // -----------------------------------------------------
                        // 4. DEVICE_ID
                        // -----------------------------------------------------
                        ST_READ_DEVICE: begin
                            device_id      <= rx_data;
                            crc_data       <= rx_data;
                            crc_data_valid <= 1'b1;
                            state          <= ST_READ_COMMAND;
                        end

                        // -----------------------------------------------------
                        // 5. CMD
                        // -----------------------------------------------------
                        ST_READ_COMMAND: begin
                            command        <= rx_data;
                            crc_data       <= rx_data;
                            crc_data_valid <= 1'b1;
                            state          <= ST_READ_SEQ;
                        end

                        // -----------------------------------------------------
                        // 6. SEQ
                        // -----------------------------------------------------
                        ST_READ_SEQ: begin
                            sequence       <= rx_data;
                            crc_data       <= rx_data;
                            crc_data_valid <= 1'b1;

                            if (payload_remaining == 5'd0)
                                state <= ST_READ_CRC_H;
                            else
                                state <= ST_READ_PAYLOAD;
                        end

                        // -----------------------------------------------------
                        // 7. PAYLOAD 0~16바이트
                        // -----------------------------------------------------
                        ST_READ_PAYLOAD: begin
                            // 먼저 들어온 바이트가 상위 쪽에 오도록 이어 붙인다.
                            // 예: DE, AD 수신 -> payload_data[15:0] = 16'hDEAD
                            payload_data   <= {payload_data[119:0], rx_data};
                            crc_data       <= rx_data;
                            crc_data_valid <= 1'b1;

                            if (payload_remaining == 5'd1) begin
                                payload_remaining <= 5'd0;
                                state             <= ST_READ_CRC_H;
                            end
                            else begin
                                payload_remaining <= payload_remaining - 1'b1;
                            end
                        end

                        // -----------------------------------------------------
                        // 8. CRC High Byte
                        // -----------------------------------------------------
                        ST_READ_CRC_H: begin
                            received_crc[15:8] <= rx_data;
                            state              <= ST_READ_CRC_L;
                        end

                        // -----------------------------------------------------
                        // 9. CRC Low Byte / 프레임 조립 완료
                        // -----------------------------------------------------
                        ST_READ_CRC_L: begin
                            received_crc[7:0] <= rx_data;
                            packet_valid      <= 1'b1;

                            // 다음 프레임의 SYNC1을 다시 찾는다.
                            state             <= ST_WAIT_SYNC1;
                            payload_remaining <= 5'd0;
                            interbyte_count   <= {INTERBYTE_CNT_WIDTH{1'b0}};
                            frame_count       <= {FRAME_CNT_WIDTH{1'b0}};
                        end

                        default: begin
                            state             <= ST_WAIT_SYNC1;
                            payload_remaining <= 5'd0;
                            interbyte_count   <= {INTERBYTE_CNT_WIDTH{1'b0}};
                            frame_count       <= {FRAME_CNT_WIDTH{1'b0}};
                        end
                    endcase
                end
            end
        end
    end

endmodule
