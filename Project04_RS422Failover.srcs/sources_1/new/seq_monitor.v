`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// seq_monitor
//
// CRC 검사를 통과한 프레임의 8비트 SEQ를 직전 기준값과 비교하여
// 정상, 중복, 누락(Gap), 과거/지연(Old) 프레임으로 분류한다.
//
// 연결 기준
//   seq_valid = crc_done && crc_ok
//   seq_value = frame_parser의 sequence
//
// 판정 방법
//   forward_distance = seq_value - last_rx_seq
//
//   최초 수신 : 비교 기준 등록, Accept
//   거리 0    : Duplicate, 폐기
//   거리 1    : Normal, Accept
//   거리 2~127: Gap, Accept 후 현재 SEQ로 재동기화
//   거리 128~255: Old/Late, 폐기
//
// 8비트 뺄셈은 자동으로 0~255 범위에서 순환하므로
// 8'hFF 다음 8'h00의 forward_distance도 8'd1이 된다.
//
// 출력 규칙
//   seq_accept, seq_ok, seq_duplicate, seq_gap, seq_old는 모두 1클럭 펄스이다.
//   seq_accept=1인 프레임만 다음 모듈의 후보 프레임으로 전달한다.
//   Gap은 오류 사실을 seq_gap으로 알리지만, 이중화 비교를 위해 전달한다.
//////////////////////////////////////////////////////////////////////////////////

module seq_monitor (
    input  wire       clk,
    input  wire       reset_p,
    input  wire       clear,

    // CRC가 정상인 프레임이 준비되었음을 알리는 1클럭 펄스
    input  wire       seq_valid,

    // 현재 프레임에 들어 있는 8비트 SEQ
    input  wire [7:0] seq_value,

    // 현재 프레임을 다음 단계로 전달할지 나타내는 1클럭 펄스
    output reg        seq_accept,

    // 판정 결과: 한 번의 seq_valid에 아래 네 신호 중 하나만 1이 된다.
    // 최초 프레임은 오류가 없으므로 seq_ok=1로 처리한다.
    output reg        seq_ok,
    output reg        seq_duplicate,
    output reg        seq_gap,
    output reg        seq_old,

    // 디버깅 및 상태 확인용
    output reg  [7:0] last_rx_seq,
    output reg        seq_initialized
);

    // 현재 SEQ가 직전 기준값보다 몇 칸 앞에 있는지 나타낸다.
    // 하위 8비트만 사용하므로 8'hFF -> 8'h00도 거리 1로 계산된다.
    wire [7:0] forward_distance;
    assign forward_distance = seq_value - last_rx_seq;

    // -------------------------------------------------------------------------
    // SEQ 기준값 저장 및 프레임 분류
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            seq_accept      <= 1'b0;
            seq_ok          <= 1'b0;
            seq_duplicate   <= 1'b0;
            seq_gap         <= 1'b0;
            seq_old         <= 1'b0;
            last_rx_seq     <= 8'h00;
            seq_initialized <= 1'b0;
        end
        else if (clear) begin
            seq_accept      <= 1'b0;
            seq_ok          <= 1'b0;
            seq_duplicate   <= 1'b0;
            seq_gap         <= 1'b0;
            seq_old         <= 1'b0;
            last_rx_seq     <= 8'h00;
            seq_initialized <= 1'b0;
        end
        else begin
            // 판정 결과는 사건이 발생한 순간만 알리는 1클럭 펄스이다.
            seq_accept    <= 1'b0;
            seq_ok        <= 1'b0;
            seq_duplicate <= 1'b0;
            seq_gap       <= 1'b0;
            seq_old       <= 1'b0;

            // seq_valid=1일 때만 현재 SEQ를 검사한다.
            if (seq_valid) begin
                if (!seq_initialized) begin
                    // 첫 프레임은 비교 대상이 없으므로 현재값을 기준으로 등록한다.
                    last_rx_seq     <= seq_value;
                    seq_initialized <= 1'b1;
                    seq_accept      <= 1'b1;
                    seq_ok          <= 1'b1;
                end
                else if (forward_distance == 8'd0) begin
                    // 직전 기준값과 같으므로 중복 프레임이다.
                    // 기준값은 바꾸지 않고 다음 단계로도 전달하지 않는다.
                    seq_duplicate <= 1'b1;
                end
                else if (forward_distance == 8'd1) begin
                    // 정확히 다음 번호이므로 정상 프레임이다.
                    last_rx_seq <= seq_value;
                    seq_accept  <= 1'b1;
                    seq_ok      <= 1'b1;
                end
                else if (forward_distance < 8'd128) begin
                    // 거리 2~127: 중간 번호가 빠진 Gap이다.
                    // Gap 사실은 기록하되 현재 프레임은 후보로 전달하고,
                    // 기준값을 현재 SEQ로 갱신하여 다음 프레임부터 복구한다.
                    last_rx_seq <= seq_value;
                    seq_accept  <= 1'b1;
                    seq_gap     <= 1'b1;
                end
                else begin
                    // 거리 128~255: 이전 번호가 늦게 도착한 것으로 처리한다.
                    // 거리 128은 앞/뒤를 구분할 수 없는 경계이므로 Old로 분류한다.
                    // 기준값은 바꾸지 않고 다음 단계로도 전달하지 않는다.
                    seq_old <= 1'b1;
                end
            end
        end
    end

endmodule
