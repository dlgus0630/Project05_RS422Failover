`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// uart_tx
//
// 역할
//   raw_frame_buffer가 제공하는 1Byte 데이터를 UART 8-N-1 형식으로
//   직렬 송신한다.
//
// Frame 형식
//   Idle(1) -> Start(0) -> Data[0] ... Data[7] -> Stop(1)
//
// Handshake
//   tx_valid && tx_ready인 상승 에지에서 tx_data를 1회 받아 송신한다.
//   송신 중에는 tx_ready=0이므로 상위 모듈은 현재 tx_data를 유지한다.
//
// 기본 설정
//   Clock : 100 MHz
//   Baud  : 115200
//   Data  : 8 bit, LSB first
//   Parity: None
//   Stop  : 1 bit
//////////////////////////////////////////////////////////////////////////////////

module uart_tx #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD_RATE   = 115_200
)(
    input  wire       clk,
    input  wire       reset_p,
    input  wire       clear,

    // raw_frame_buffer와 연결되는 Byte 입력
    input  wire       tx_valid,
    output wire       tx_ready,
    input  wire [7:0] tx_data,

    // UART 물리 출력
    output reg        uart_txd,

    // 상태 및 Event
    output reg        tx_busy,
    output reg        tx_done
);

    // 반올림한 Clock 수를 한 UART Bit의 길이로 사용한다.
    localparam integer CALCULATED_CLKS_PER_BIT =
        (CLK_FREQ_HZ + (BAUD_RATE / 2)) / BAUD_RATE;

    // 잘못된 조합에서 Counter가 0주기가 되지 않도록 최소값을 1로 제한한다.
    localparam integer CLKS_PER_BIT =
        (CALCULATED_CLKS_PER_BIT < 1) ? 1 : CALCULATED_CLKS_PER_BIT;

    localparam integer BAUD_COUNT_WIDTH =
        (CLKS_PER_BIT <= 1) ? 1 : $clog2(CLKS_PER_BIT);

    reg [BAUD_COUNT_WIDTH-1:0] baud_count;
    reg [3:0]                  bit_index;
    reg [7:0]                  shift_reg;

    // Reset 중에는 입력을 받지 않고, Idle 상태에서만 새 Byte를 받는다.
    assign tx_ready = !reset_p && !clear && !tx_busy;

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            uart_txd  <= 1'b1;
            tx_busy   <= 1'b0;
            tx_done   <= 1'b0;
            baud_count <= {BAUD_COUNT_WIDTH{1'b0}};
            bit_index <= 4'd0;
            shift_reg <= 8'd0;
        end
        else if (clear) begin
            uart_txd   <= 1'b1;
            tx_busy    <= 1'b0;
            tx_done    <= 1'b0;
            baud_count <= {BAUD_COUNT_WIDTH{1'b0}};
            bit_index  <= 4'd0;
            shift_reg  <= 8'd0;
        end
        else begin
            // Byte 하나의 Stop Bit까지 끝난 순간에만 1클럭 Pulse가 된다.
            tx_done <= 1'b0;

            if (!tx_busy) begin
                // UART의 기본 대기 상태는 High이다.
                uart_txd   <= 1'b1;
                baud_count <= {BAUD_COUNT_WIDTH{1'b0}};
                bit_index  <= 4'd0;

                if (tx_valid) begin
                    // Handshake 시점의 데이터만 저장한다.
                    shift_reg <= tx_data;
                    tx_busy   <= 1'b1;

                    // Start Bit를 즉시 시작한다.
                    uart_txd <= 1'b0;
                end
            end
            else begin
                if (baud_count == CLKS_PER_BIT - 1) begin
                    baud_count <= {BAUD_COUNT_WIDTH{1'b0}};

                    if (bit_index < 4'd8) begin
                        // Start Bit 뒤에 Data[0]부터 LSB-first로 출력한다.
                        uart_txd  <= shift_reg[bit_index];
                        bit_index <= bit_index + 1'b1;
                    end
                    else if (bit_index == 4'd8) begin
                        // 8개 Data Bit 뒤의 Stop Bit
                        uart_txd  <= 1'b1;
                        bit_index <= 4'd9;
                    end
                    else begin
                        // Stop Bit 한 주기가 모두 끝난 시점
                        uart_txd <= 1'b1;
                        tx_busy  <= 1'b0;
                        tx_done  <= 1'b1;
                        bit_index <= 4'd0;
                    end
                end
                else begin
                    baud_count <= baud_count + 1'b1;
                end
            end
        end
    end

endmodule
