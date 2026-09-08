`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// crc16_ccitt
//
// frame_parser가 LEN부터 마지막 PAYLOAD까지 한 바이트씩 전달하면
// CRC-16/CCITT-FALSE 값을 수신 중에 누적 계산한다.
//
// CRC 규격
//   다항식        : 16'h1021
//   초기값        : 16'hFFFF
//   입력 비트 반전: 없음
//   출력 비트 반전: 없음
//   최종 XOR      : 16'h0000
//
// frame_parser와의 연결
//   crc_data        <- crc_data
//   crc_data_valid  <- crc_data_valid
//   crc_start       <- crc_start
//   packet_valid    <- packet_valid
//   received_crc    <- received_crc
//
// 동작 순서
//   1. LEN 바이트에서 crc_start와 crc_data_valid가 함께 1이 된다.
//      기존 CRC를 16'hFFFF로 초기화한 뒤 LEN을 바로 계산한다.
//   2. ID, CMD, SEQ, PAYLOAD가 들어올 때마다 CRC를 한 바이트씩 갱신한다.
//   3. CRC_H/L은 계산에 포함하지 않는다.
//   4. packet_valid가 1이면 calculated_crc와 received_crc를 비교한다.
//   5. crc_done과 crc_ok 또는 crc_error를 1클럭 펄스로 출력한다.
//////////////////////////////////////////////////////////////////////////////////

module crc16_ccitt (
    input  wire        clk,
    input  wire        reset_p,
    input  wire        clear,

    // frame_parser가 LEN~PAYLOAD를 전달하는 스트리밍 입력
    input  wire [7:0]  crc_data,
    input  wire        crc_data_valid,
    input  wire        crc_start,

    // frame_parser가 CRC_L까지 저장한 뒤 발생시키는 완료 신호와 수신 CRC
    input  wire        packet_valid,
    input  wire [15:0] received_crc,

    // 계산 결과와 판정 펄스
    output reg  [15:0] calculated_crc,
    output reg         crc_done,
    output reg         crc_ok,
    output reg         crc_error
);

    localparam [15:0] CRC_INITIAL = 16'hFFFF;
    localparam [15:0] CRC_POLY    = 16'h1021;

    // 현재 바이트를 반영한 다음 CRC 값이다.
    // 한 바이트를 한 클럭에 처리하기 위해 8비트 계산을 조합회로로 펼친다.
    reg [15:0] crc_next;
    integer bit_index;

    // -------------------------------------------------------------------------
    // CRC-16/CCITT-FALSE 한 바이트 계산
    //
    // crc_start=1이면 이전 계산값 대신 16'hFFFF에서 새 프레임을 시작한다.
    // MSB-first 방식이므로 입력 바이트를 CRC 상위 8비트에 XOR한 뒤,
    // 8번 Shift하면서 최상위 비트가 1일 때 16'h1021을 XOR한다.
    // -------------------------------------------------------------------------
    always @* begin
        if (crc_start)
            crc_next = CRC_INITIAL;
        else
            crc_next = calculated_crc;

        crc_next = crc_next ^ {crc_data, 8'h00};

        for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
            if (crc_next[15])
                crc_next = {crc_next[14:0], 1'b0} ^ CRC_POLY;
            else
                crc_next = {crc_next[14:0], 1'b0};
        end
    end

    // -------------------------------------------------------------------------
    // CRC 누적 레지스터와 최종 비교
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            calculated_crc <= CRC_INITIAL;
            crc_done       <= 1'b0;
            crc_ok         <= 1'b0;
            crc_error      <= 1'b0;
        end
        else if (clear) begin
            calculated_crc <= 16'hFFFF;
            crc_done      <= 1'b0;
            crc_ok        <= 1'b0;
            crc_error     <= 1'b0;
        end
        else begin
            // 아래 세 신호는 판정이 끝난 순간만 알리는 1클럭 펄스이다.
            crc_done  <= 1'b0;
            crc_ok    <= 1'b0;
            crc_error <= 1'b0;

            // 새 프레임 시작 신호만 들어오면 초기값으로 복귀한다.
            // 현재 frame_parser에서는 crc_start와 crc_data_valid가
            // LEN 바이트에서 항상 함께 들어온다.
            if (crc_start && !crc_data_valid)
                calculated_crc <= CRC_INITIAL;
            else if (crc_data_valid)
                calculated_crc <= crc_next;

            // packet_valid는 마지막 계산 바이트보다 뒤에 발생한다.
            // 따라서 이 시점의 calculated_crc가 LEN~PAYLOAD의 최종 CRC이다.
            if (packet_valid) begin
                crc_done <= 1'b1;

                if (calculated_crc == received_crc) begin
                    crc_ok    <= 1'b1;
                    crc_error <= 1'b0;
                end
                else begin
                    crc_ok    <= 1'b0;
                    crc_error <= 1'b1;
                end
            end
        end
    end

endmodule
