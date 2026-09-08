`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// [이 모듈은 UART RX 선으로 들어오는 직렬 비트를 받아
//  8비트 데이터 1바이트로 복원하는 모듈이다.]
//
// UART 설정
//   - 통신 형식 : 8-N-1
//                 Data 8비트 / Parity 없음 / Stop Bit 1개
//   - 전송 순서 : LSB First
//                 예) 8'hA5를 보낼 때 bit[0]부터 차례대로 수신
//   - 대기 상태 : RX = High
//
// UART 한 바이트의 선로 흐름
//
//   Idle(1) -> Start(0) -> D0 -> D1 -> ... -> D7 -> Stop(1)
//
// 이 모듈의 처리 순서
//   1. 비동기 입력 rx를 2개의 플립플롭으로 동기화한다.
//   2. RX가 Low가 되면 Start Bit 후보로 판단한다.
//   3. Start Bit 중앙에서도 Low인지 다시 확인한다.
//   4. 이후 한 비트 시간마다 Data Bit 8개를 중앙에서 샘플링한다.
//   5. Stop Bit가 High이면 rx_data를 갱신하고 rx_valid를 1클럭 출력한다.
//   6. Stop Bit가 Low이면 데이터를 폐기하고 rx_frame_error를 1클럭 출력한다.
//
// 주의
//   - 이 모듈은 '패킷' 전체를 해석하지 않는다.
//   - UART 직렬 신호를 1바이트로 복원하는 역할만 한다.
//   - 패킷의 SYNC, LEN, SEQ, CRC 등은 뒤쪽 frame_parser가 처리한다.
//////////////////////////////////////////////////////////////////////////////////

module uart_rx #(
    // FPGA의 시스템 클럭 주파수와 UART 통신 속도이다.
    // Basys3 기본 설정: 100 MHz / 115200 bps
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD_RATE   = 115_200
)(
    input  wire       clk,             // FPGA 시스템 클럭
    input  wire       reset_p,         // Active-High 비동기 리셋
    input  wire       clear,           // Active-High 동기 Clear
    input  wire       rx,              // 외부 UART 수신선(Idle 상태는 High)

    output reg  [7:0] rx_data,         // 정상적으로 복원된 수신 데이터 1바이트
    output reg        rx_valid,        // 정상 바이트 수신 완료 알림(1클럭 High)
    output reg        rx_frame_error   // Stop Bit 오류 알림(1클럭 High)
);

    // -------------------------------------------------------------------------
    // Baud Rate용 클럭 수 계산
    // -------------------------------------------------------------------------
    // 100 MHz / 115200 bps = 약 868클럭마다 UART 1비트가 진행된다.
    // 정수 나눗셈이므로 소수점 이하는 버려지지만, 이 설정에서는 오차가 작다.
    localparam integer CLKS_PER_BIT      = CLK_FREQ_HZ / BAUD_RATE;

    // Start Bit의 중앙을 확인하기 위한 반 비트 시간이다.
    // 100 MHz / 115200 bps 설정에서는 434클럭이다.
    localparam integer HALF_CLKS_PER_BIT = CLKS_PER_BIT / 2;

    // baud_count가 0부터 CLKS_PER_BIT-1까지 셀 수 있도록
    // 필요한 레지스터 비트 수를 자동으로 계산한다.
    localparam integer BAUD_CNT_WIDTH     =
        (CLKS_PER_BIT <= 2) ? 1 : $clog2(CLKS_PER_BIT);

    // -------------------------------------------------------------------------
    // UART 수신 FSM 상태 정의
    // -------------------------------------------------------------------------
    // ST_IDLE      : RX가 Low로 내려가 Start Bit가 시작되기를 기다린다.
    // ST_START     : Start Bit 중앙에서도 RX가 Low인지 확인한다.
    // ST_DATA      : Data Bit 8개를 LSB부터 차례대로 수신한다.
    // ST_STOP      : Stop Bit가 High인지 확인한다.
    // ST_WAIT_IDLE : 오류 후 RX가 다시 High가 될 때까지 기다린다.
    localparam [2:0] ST_IDLE      = 3'd0;
    localparam [2:0] ST_START     = 3'd1;
    localparam [2:0] ST_DATA      = 3'd2;
    localparam [2:0] ST_STOP      = 3'd3;
    localparam [2:0] ST_WAIT_IDLE = 3'd4;

    // 현재 UART 수신 FSM 상태
    reg [2:0] state;

    // 현재 수신 중인 Data Bit 번호: 0부터 7까지 증가한다.
    reg [2:0] bit_index;

    // Data Bit를 하나씩 임시 저장하는 시프트 레지스터이다.
    // UART가 LSB First이므로 rx_shift[0]부터 채운다.
    reg [7:0] rx_shift;

    // 각 UART Bit의 중앙 샘플링 시점을 만들기 위한 클럭 카운터이다.
    reg [BAUD_CNT_WIDTH-1:0] baud_count;

    // -------------------------------------------------------------------------
    // 비동기 RX 입력 동기화
    // -------------------------------------------------------------------------
    // 외부 장치의 rx 신호는 FPGA의 clk와 타이밍 관계가 없는 비동기 신호다.
    // 따라서 rx를 곧바로 FSM에 넣지 않고 플립플롭 2개를 거쳐 사용한다.
    // 이 구조는 첫 번째 플립플롭에서 발생할 수 있는 Metastability가
    // 뒤쪽 로직으로 전달될 가능성을 낮춘다.
    //
    // UART의 대기 상태가 High이므로 두 플립플롭의 리셋값도 1로 둔다.
    // ASYNC_REG 속성은 Vivado에 이 레지스터들이 동기화용임을 알려준다.
    (* ASYNC_REG = "TRUE" *) reg [1:0] rx_sync_ff;
    wire rx_sync;

    // FSM은 외부 rx가 아니라 2단 동기화를 마친 rx_sync를 사용한다.
    assign rx_sync = rx_sync_ff[1];

    // 2-FF 동기화 회로
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            rx_sync_ff <= 2'b11;
        end
        else if (clear) begin
            rx_sync_ff <= 2'b11;
        end
        else begin
            rx_sync_ff[0] <= rx;            // 1단: 외부 비동기 신호 수신
            rx_sync_ff[1] <= rx_sync_ff[0]; // 2단: FSM에 전달할 안정된 신호
        end
    end

    // -------------------------------------------------------------------------
    // UART 수신 FSM
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            // 리셋 시 수신 대기 상태로 돌아가고 내부 값을 초기화한다.
            state          <= ST_IDLE;
            baud_count     <= {BAUD_CNT_WIDTH{1'b0}};
            bit_index      <= 3'd0;
            rx_shift       <= 8'd0;
            rx_data        <= 8'd0;
            rx_valid       <= 1'b0;
            rx_frame_error <= 1'b0;
        end
        else if (clear) begin
            state          <= ST_IDLE;
            baud_count     <= {BAUD_CNT_WIDTH{1'b0}};
            bit_index      <= 3'd0;
            rx_shift       <= 8'd0;
            rx_data        <= 8'd0;
            rx_valid       <= 1'b0;
            rx_frame_error <= 1'b0;
        end
        else begin
            // rx_valid와 rx_frame_error는 '상태값'이 아니라 '발생 알림'이다.
            // 따라서 매 클럭 기본값을 0으로 만들고, 이벤트가 발생한
            // 해당 클럭에만 아래 FSM에서 1로 올린다.
            rx_valid       <= 1'b0;
            rx_frame_error <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // 1. 수신 대기
                // -------------------------------------------------------------
                ST_IDLE: begin
                    baud_count <= {BAUD_CNT_WIDTH{1'b0}};
                    bit_index  <= 3'd0;

                    // UART 선이 High에서 Low로 내려가면 Start Bit일 수 있다.
                    // 아직 확정하지 않고 ST_START에서 중앙을 다시 확인한다.
                    if (rx_sync == 1'b0) begin
                        state <= ST_START;
                    end
                end

                // -------------------------------------------------------------
                // 2. Start Bit 중앙 확인
                // -------------------------------------------------------------
                ST_START: begin
                    // Low를 처음 감지한 뒤 반 비트만큼 기다린다.
                    // 이 시점은 Start Bit의 중앙 부근이다.
                    if (baud_count == HALF_CLKS_PER_BIT - 1) begin
                        baud_count <= {BAUD_CNT_WIDTH{1'b0}};

                        if (rx_sync == 1'b0) begin
                            // 중앙에서도 Low이면 정상 Start Bit로 인정한다.
                            // 다음부터 1비트 간격으로 Data Bit를 읽는다.
                            state     <= ST_DATA;
                            bit_index <= 3'd0;
                            rx_shift  <= 8'd0;
                        end
                        else begin
                            // 중앙에 도달하기 전에 High로 돌아왔다면
                            // 짧은 노이즈였으므로 False Start로 보고 무시한다.
                            state <= ST_IDLE;
                        end
                    end
                    else begin
                        // 아직 Start Bit 중앙이 아니므로 클럭 수를 센다.
                        baud_count <= baud_count + 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // 3. Data Bit 8개 수신
                // -------------------------------------------------------------
                ST_DATA: begin
                    // Start Bit 중앙에서 한 비트 시간을 더 기다리면
                    // 첫 번째 Data Bit(D0)의 중앙에 도달한다.
                    // 이후에도 한 비트 시간마다 D1~D7의 중앙을 샘플링한다.
                    if (baud_count == CLKS_PER_BIT - 1) begin
                        baud_count         <= {BAUD_CNT_WIDTH{1'b0}};

                        // UART는 LSB First이므로 첫 비트를 rx_shift[0]에,
                        // 마지막 비트를 rx_shift[7]에 저장한다.
                        rx_shift[bit_index] <= rx_sync;

                        if (bit_index == 3'd7) begin
                            // D7까지 받았으면 다음은 Stop Bit이다.
                            state <= ST_STOP;
                        end
                        else begin
                            // 아직 8비트를 다 받지 않았으면 다음 Bit로 이동한다.
                            bit_index <= bit_index + 1'b1;
                        end
                    end
                    else begin
                        // 다음 Data Bit 중앙까지 클럭 수를 센다.
                        baud_count <= baud_count + 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // 4. Stop Bit 검사 및 결과 출력
                // -------------------------------------------------------------
                ST_STOP: begin
                    // D7 중앙에서 한 비트 시간을 기다리면 Stop Bit 중앙이다.
                    // 정상적인 Stop Bit는 반드시 High여야 한다.
                    if (baud_count == CLKS_PER_BIT - 1) begin
                        baud_count <= {BAUD_CNT_WIDTH{1'b0}};

                        if (rx_sync == 1'b1) begin
                            // Stop Bit가 정상이므로 완성된 1바이트를 출력한다.
                            rx_data  <= rx_shift;

                            // rx_valid는 현재 클럭에서만 1이 된다.
                            // 뒤쪽 모듈은 rx_valid가 1일 때 rx_data를 읽는다.
                            rx_valid <= 1'b1;
                            state    <= ST_IDLE;
                        end
                        else begin
                            // Stop Bit가 Low이면 UART Framing Error이다.
                            // 이때 rx_shift의 값은 정상 데이터로 출력하지 않는다.
                            rx_frame_error <= 1'b1;
                            state          <= ST_WAIT_IDLE;
                        end
                    end
                    else begin
                        // Stop Bit 중앙까지 클럭 수를 센다.
                        baud_count <= baud_count + 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // 5. Framing Error 후 선로 복귀 대기
                // -------------------------------------------------------------
                ST_WAIT_IDLE: begin
                    // 오류 직후 RX가 계속 Low라면 이를 새로운 Start Bit로
                    // 반복 인식하면 안 된다. RX가 UART 대기값인 High로
                    // 돌아온 뒤에만 새로운 바이트 수신을 허용한다.
                    baud_count <= {BAUD_CNT_WIDTH{1'b0}};
                    bit_index  <= 3'd0;

                    if (rx_sync == 1'b1) begin
                        state <= ST_IDLE;
                    end
                end

                // 정의되지 않은 상태에 들어간 경우 안전하게 초기 상태로 복귀한다.
                default: begin
                    state          <= ST_IDLE;
                    baud_count     <= {BAUD_CNT_WIDTH{1'b0}};
                    bit_index      <= 3'd0;
                    rx_shift       <= 8'd0;
                    rx_valid       <= 1'b0;
                    rx_frame_error <= 1'b0;
                end
            endcase
        end
    end

endmodule
