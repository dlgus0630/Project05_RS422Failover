`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// channel_health_mgr
//
// 역할
//   1. pair_matcher 결과를 A/B 채널의 Good/Fail 사건으로 변환한다.
//   2. 정상 채널은 연속 Fail 3회에서 Fault로 전환한다.
//   3. Fault 채널은 정상 Pair 5회에서 Healthy로 복구한다.
//
// matcher_result_kind
//   2'b00 : 결과 없음
//   2'b01 : Pair
//   2'b10 : Single A  -> A 정상, B 누락
//   2'b11 : Single B  -> A 누락, B 정상
//
// local_fail_event
//   CRC 오류, SEQ 오류, FIFO Overflow, 300ms Channel Timeout처럼
//   pair_matcher 밖에서 검출한 채널별 오류를 1클럭 Pulse로 입력한다.
//   같은 논리적 오류를 중복 Pulse로 넣지 않아야 한다.
//
// 중요
//   WAIT_SYNC/DUAL/SINGLE_A/SINGLE_B/BOTH_FAULT 상태와 출력 선택은
//   다음 단계인 decision_unit에서 관리한다.
// -----------------------------------------------------------------------------

module channel_health_mgr #(
    parameter integer FAIL_THRESHOLD    = 3,
    parameter integer RECOVER_THRESHOLD = 5
)(
    input  wire       clk,
    input  wire       reset_p,
    input  wire       clear,

    // pair_matcher 결과
    input  wire       matcher_result_valid,
    input  wire [1:0] matcher_result_kind,
    input  wire       matcher_pair_equal,
    input  wire       matcher_result_a_seq_gap,
    input  wire       matcher_result_b_seq_gap,

    // pair_matcher 외부에서 발생한 채널별 오류 Pulse
    input  wire       a_local_fail_event,
    input  wire       b_local_fail_event,

    // AXI-Lite에서 설정하는 런타임 임계값. 0은 Parameter 기본값을 쓴다.
    input  wire [7:0] fail_threshold_cfg,
    input  wire [7:0] recover_threshold_cfg,

    // decision_unit과 AXI Status에서 사용할 상태
    output reg        a_fault,
    output reg        b_fault,
    output reg  [7:0] a_fail_count,
    output reg  [7:0] b_fail_count,
    output reg  [7:0] a_recover_count,
    output reg  [7:0] b_recover_count,

    // 상태가 바뀌는 순간에만 1클럭 Pulse
    output reg        a_fault_enter_pulse,
    output reg        b_fault_enter_pulse,
    output reg        a_recovered_pulse,
    output reg        b_recovered_pulse
);

    localparam [1:0] RESULT_NONE     = 2'b00;
    localparam [1:0] RESULT_PAIR     = 2'b01;
    localparam [1:0] RESULT_SINGLE_A = 2'b10;
    localparam [1:0] RESULT_SINGLE_B = 2'b11;

    wire [7:0] effective_fail_threshold;
    wire [7:0] effective_recover_threshold;

    assign effective_fail_threshold =
        (fail_threshold_cfg == 8'd0) ?
        FAIL_THRESHOLD[7:0] : fail_threshold_cfg;
    assign effective_recover_threshold =
        (recover_threshold_cfg == 8'd0) ?
        RECOVER_THRESHOLD[7:0] : recover_threshold_cfg;

    // 이번 matcher 결과가 각 채널에 대해 Good인지 Fail인지 나타낸다.
    reg a_good_event;
    reg b_good_event;
    reg a_bad_event;
    reg b_bad_event;
    wire a_recovery_match_event;
    wire b_recovery_match_event;

    // Fault 복구는 "동일 Pair + 내용 일치"일 때만 누적한다.
    // 조합 always 블록 안에서 쓰고 다시 읽는 임시 reg를 피하여
    // 시뮬레이션 delta-cycle에 관계없이 확정된 값을 순차부에 전달한다.
    assign a_recovery_match_event =
        matcher_result_valid &&
        (matcher_result_kind == RESULT_PAIR) &&
        matcher_pair_equal &&
        !matcher_result_a_seq_gap &&
        !a_local_fail_event;

    assign b_recovery_match_event =
        matcher_result_valid &&
        (matcher_result_kind == RESULT_PAIR) &&
        matcher_pair_equal &&
        !matcher_result_b_seq_gap &&
        !b_local_fail_event;

    always @(*) begin
        a_good_event = 1'b0;
        b_good_event = 1'b0;
        a_bad_event  = a_local_fail_event;
        b_bad_event  = b_local_fail_event;

        if (matcher_result_valid) begin
            case (matcher_result_kind)
                RESULT_PAIR: begin
                    if (matcher_pair_equal) begin
                        // GAP 채널은 앞에서 이미 Fail 1회가 반영됐으므로
                        // 이 Pair 결과에서는 Good으로 계산하지 않는다.
                        if (!matcher_result_a_seq_gap)
                            a_good_event = 1'b1;

                        if (!matcher_result_b_seq_gap)
                            b_good_event = 1'b1;
                    end
                    else begin
                        // GAP 채널은 중복 Fail을 막기 위해 Neutral 처리한다.
                        if (!matcher_result_a_seq_gap)
                            a_bad_event = 1'b1;

                        if (!matcher_result_b_seq_gap)
                            b_bad_event = 1'b1;
                    end
                end

                RESULT_SINGLE_A: begin
                    // A GAP이면 A는 Neutral, 반대편 B는 누락 Fail이다.
                    if (!matcher_result_a_seq_gap)
                        a_good_event = 1'b1;

                    b_bad_event = 1'b1;
                end

                RESULT_SINGLE_B: begin
                    // B GAP이면 B는 Neutral, 반대편 A는 누락 Fail이다.
                    a_bad_event = 1'b1;

                    if (!matcher_result_b_seq_gap)
                        b_good_event = 1'b1;
                end

                RESULT_NONE: begin
                    // 동작 없음
                end

                default: begin
                    // 동작 없음
                end
            endcase
        end

        // 같은 클럭에 Good과 Local Fail이 겹치면 Fail을 우선한다.
        if (a_bad_event)
            a_good_event = 1'b0;

        if (b_bad_event)
            b_good_event = 1'b0;
    end

    // -------------------------------------------------------------------------
    // Channel A 상태 관리
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            a_fault             <= 1'b0;
            a_fail_count        <= 8'd0;
            a_recover_count     <= 8'd0;
            a_fault_enter_pulse <= 1'b0;
            a_recovered_pulse   <= 1'b0;
        end
        else if (clear) begin
            a_fault             <= 1'b0;
            a_fail_count        <= 8'd0;
            a_recover_count     <= 8'd0;
            a_fault_enter_pulse <= 1'b0;
            a_recovered_pulse   <= 1'b0;
        end
        else begin
            a_fault_enter_pulse <= 1'b0;
            a_recovered_pulse   <= 1'b0;

            if (a_bad_event) begin
                // Fail이 발생하면 복구 연속 횟수는 끊긴다.
                a_recover_count <= 8'd0;

                if (!a_fault) begin
                    if (a_fail_count >=
                        (effective_fail_threshold - 1'b1)) begin
                        a_fault             <= 1'b1;
                        a_fail_count        <= effective_fail_threshold;
                        a_fault_enter_pulse <= 1'b1;
                    end
                    else begin
                        a_fail_count <= a_fail_count + 1'b1;
                    end
                end
            end
            else if (a_fault) begin
                if (a_recovery_match_event) begin
                    // Fault 채널은 정상 Pair가 연속 5회 들어와야 복구한다.
                    if (a_recover_count >=
                        (effective_recover_threshold - 1'b1)) begin
                        a_fault           <= 1'b0;
                        a_fail_count      <= 8'd0;
                        a_recover_count   <= 8'd0;
                        a_recovered_pulse <= 1'b1;
                    end
                    else begin
                        a_recover_count <= a_recover_count + 1'b1;
                    end
                end
                else if (matcher_result_valid) begin
                    // Pair 일치가 아닌 Matcher 결과는 복구 연속성을 끊는다.
                    a_recover_count <= 8'd0;
                end
            end
            else if (a_good_event) begin
                // 정상 채널의 Good 사건은 연속 Fail 횟수를 초기화한다.
                a_fail_count    <= 8'd0;
                a_recover_count <= 8'd0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Channel B 상태 관리
    // -------------------------------------------------------------------------
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            b_fault             <= 1'b0;
            b_fail_count        <= 8'd0;
            b_recover_count     <= 8'd0;
            b_fault_enter_pulse <= 1'b0;
            b_recovered_pulse   <= 1'b0;
        end
        else if (clear) begin
            b_fault             <= 1'b0;
            b_fail_count        <= 8'd0;
            b_recover_count     <= 8'd0;
            b_fault_enter_pulse <= 1'b0;
            b_recovered_pulse   <= 1'b0;
        end
        else begin
            b_fault_enter_pulse <= 1'b0;
            b_recovered_pulse   <= 1'b0;

            if (b_bad_event) begin
                // Fail이 발생하면 복구 연속 횟수는 끊긴다.
                b_recover_count <= 8'd0;

                if (!b_fault) begin
                    if (b_fail_count >=
                        (effective_fail_threshold - 1'b1)) begin
                        b_fault             <= 1'b1;
                        b_fail_count        <= effective_fail_threshold;
                        b_fault_enter_pulse <= 1'b1;
                    end
                    else begin
                        b_fail_count <= b_fail_count + 1'b1;
                    end
                end
            end
            else if (b_fault) begin
                if (b_recovery_match_event) begin
                    // Fault 채널은 정상 Pair가 연속 5회 들어와야 복구한다.
                    if (b_recover_count >=
                        (effective_recover_threshold - 1'b1)) begin
                        b_fault           <= 1'b0;
                        b_fail_count      <= 8'd0;
                        b_recover_count   <= 8'd0;
                        b_recovered_pulse <= 1'b1;
                    end
                    else begin
                        b_recover_count <= b_recover_count + 1'b1;
                    end
                end
                else if (matcher_result_valid) begin
                    // Pair 일치가 아닌 Matcher 결과는 복구 연속성을 끊는다.
                    b_recover_count <= 8'd0;
                end
            end
            else if (b_good_event) begin
                // 정상 채널의 Good 사건은 연속 Fail 횟수를 초기화한다.
                b_fail_count    <= 8'd0;
                b_recover_count <= 8'd0;
            end
        end
    end

endmodule
