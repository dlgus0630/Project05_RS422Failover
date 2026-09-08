`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// event_arbiter
//
// 역할
//   여러 RTL 블록에서 같은 클럭에 발생한 Event를 소스별 Pending Slot에
//   보관하고, 한 번에 1개의 64-bit Event를 event_fifo로 전달한다.
//
// 입력 배치
//   source_valid[N]                  : Source N의 1클럭 Event Pulse
//   source_data[N*64 +: 64]          : Source N의 Event Record
//
// 출력 Handshake
//   event_valid && event_ready인 상승 에지에서 Event 1개가 전달된다.
//   event_ready=0이면 event_valid/event_data를 그대로 유지한다.
//
// 우선순위
//   번호가 작은 Source가 높은 우선순위를 가진다.
//   예: Source 0 > Source 1 > Source 2 ...
//
// Event 유실 조건
//   서로 다른 Source에서 동시에 발생한 Event는 각 Pending Slot에 저장된다.
//   단, 아직 처리되지 않은 Source에 같은 Source의 Event가 다시 들어오면
//   새 Event를 저장할 공간이 없으므로 새 Event를 버리고 event_lost_pulse를
//   발생시킨다. 기존 Pending Event는 보존한다.
//
// 중요
//   Source별 Event는 1클럭 Pulse여야 한다.
//   64-bit Event Record의 내부 필드 형식은 redundant_link_core에서 정한다.
//////////////////////////////////////////////////////////////////////////////////

module event_arbiter #(
    parameter integer SOURCE_COUNT = 8,
    parameter integer EVENT_WIDTH  = 64
)(
    input  wire                              clk,
    input  wire                              reset_p,
    input  wire                              statistics_clear,

    // 여러 Event Source의 입력
    input  wire [SOURCE_COUNT-1:0]            source_valid,
    input  wire [(SOURCE_COUNT*EVENT_WIDTH)-1:0] source_data,

    // event_fifo와 연결되는 출력
    output wire                              event_valid,
    input  wire                              event_ready,
    output reg  [EVENT_WIDTH-1:0]             event_data,

    // 상태 및 진단
    output wire                              pending_any,
    output reg  [7:0]                        pending_count,
    output reg                               event_lost_pulse,
    output reg  [15:0]                       event_lost_count
);

    // Source마다 Event 1개를 보관한다.
    reg [SOURCE_COUNT-1:0] pending_valid;
    reg [EVENT_WIDTH-1:0]  pending_data [0:SOURCE_COUNT-1];

    reg                   selected_valid;

    wire [SOURCE_COUNT-1:0] lost_vector;
    wire [SOURCE_COUNT-1:0] selected_onehot;
    wire [15:0]             lost_this_cycle;
    wire [16:0]             lost_count_sum;

    integer select_index;
    integer count_index;
    integer update_index;
    integer data_index;
    genvar  lost_gen_index;

    // -------------------------------------------------------------------------
    // 가장 높은 우선순위의 Pending Event 선택
    // -------------------------------------------------------------------------
    always @(*) begin
        selected_valid = 1'b0;
        event_data     = {EVENT_WIDTH{1'b0}};

        for (select_index = 0;
             select_index < SOURCE_COUNT;
             select_index = select_index + 1) begin

            if (!selected_valid && pending_valid[select_index]) begin
                selected_valid = 1'b1;
                event_data     = pending_data[select_index];
            end
        end
    end

    assign event_valid = selected_valid;
    assign pending_any = |pending_valid;

    // 가장 낮은 번호의 Pending Bit만 1로 만든다.
    // 우선순위 Index를 조합식으로 다시 비교하지 않아 Event 유실 Count의
    // 임계 경로를 짧게 유지한다.
    assign selected_onehot =
        pending_valid &
        (~pending_valid + {{(SOURCE_COUNT-1){1'b0}}, 1'b1});

    // -------------------------------------------------------------------------
    // 현재 Pending Event 수
    // -------------------------------------------------------------------------
    always @(*) begin
        pending_count = 8'd0;

        for (count_index = 0;
             count_index < SOURCE_COUNT;
             count_index = count_index + 1) begin

            if (pending_valid[count_index])
                pending_count = pending_count + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // 이번 클럭에 저장하지 못하는 동일 Source Event 수
    //
    // 기존 Pending Event가 이번 클럭에 FIFO로 전달되면 같은 Source의
    // 새 Event가 즉시 그 Slot을 이어받을 수 있으므로 유실이 아니다.
    // -------------------------------------------------------------------------
    generate
        for (lost_gen_index = 0;
             lost_gen_index < SOURCE_COUNT;
             lost_gen_index = lost_gen_index + 1) begin : g_lost_vector

            assign lost_vector[lost_gen_index] =
                source_valid[lost_gen_index] &&
                pending_valid[lost_gen_index] &&
                !(event_ready && selected_onehot[lost_gen_index]);
        end
    endgenerate

    // 포화 여부와 다음 Count를 한 번의 확장 덧셈으로 계산한다.
    assign lost_count_sum =
        {1'b0, event_lost_count} + {1'b0, lost_this_cycle};

    // Core의 16개 Source와 일반적인 8/4개 구성은 균형형 덧셈 트리로
    // Popcount한다. 직렬 for-loop 누산과 같은 값을 만들지만 조합 깊이는
    // log2(SOURCE_COUNT)로 줄어 100 MHz 진단 Count 경로를 만족시킨다.
    generate
        if (SOURCE_COUNT == 16) begin : g_lost_count_16
            wire [1:0] pair_0;
            wire [1:0] pair_1;
            wire [1:0] pair_2;
            wire [1:0] pair_3;
            wire [1:0] pair_4;
            wire [1:0] pair_5;
            wire [1:0] pair_6;
            wire [1:0] pair_7;
            wire [2:0] quad_0;
            wire [2:0] quad_1;
            wire [2:0] quad_2;
            wire [2:0] quad_3;
            wire [3:0] oct_0;
            wire [3:0] oct_1;
            wire [4:0] total;

            assign pair_0 = {1'b0, lost_vector[0]} +
                            {1'b0, lost_vector[1]};
            assign pair_1 = {1'b0, lost_vector[2]} +
                            {1'b0, lost_vector[3]};
            assign pair_2 = {1'b0, lost_vector[4]} +
                            {1'b0, lost_vector[5]};
            assign pair_3 = {1'b0, lost_vector[6]} +
                            {1'b0, lost_vector[7]};
            assign pair_4 = {1'b0, lost_vector[8]} +
                            {1'b0, lost_vector[9]};
            assign pair_5 = {1'b0, lost_vector[10]} +
                            {1'b0, lost_vector[11]};
            assign pair_6 = {1'b0, lost_vector[12]} +
                            {1'b0, lost_vector[13]};
            assign pair_7 = {1'b0, lost_vector[14]} +
                            {1'b0, lost_vector[15]};

            assign quad_0 = {1'b0, pair_0} + {1'b0, pair_1};
            assign quad_1 = {1'b0, pair_2} + {1'b0, pair_3};
            assign quad_2 = {1'b0, pair_4} + {1'b0, pair_5};
            assign quad_3 = {1'b0, pair_6} + {1'b0, pair_7};
            assign oct_0  = {1'b0, quad_0} + {1'b0, quad_1};
            assign oct_1  = {1'b0, quad_2} + {1'b0, quad_3};
            assign total  = {1'b0, oct_0} + {1'b0, oct_1};

            assign lost_this_cycle = {11'd0, total};
        end
        else if (SOURCE_COUNT == 8) begin : g_lost_count_8
            wire [1:0] pair_0;
            wire [1:0] pair_1;
            wire [1:0] pair_2;
            wire [1:0] pair_3;
            wire [2:0] quad_0;
            wire [2:0] quad_1;
            wire [3:0] total;

            assign pair_0 = {1'b0, lost_vector[0]} +
                            {1'b0, lost_vector[1]};
            assign pair_1 = {1'b0, lost_vector[2]} +
                            {1'b0, lost_vector[3]};
            assign pair_2 = {1'b0, lost_vector[4]} +
                            {1'b0, lost_vector[5]};
            assign pair_3 = {1'b0, lost_vector[6]} +
                            {1'b0, lost_vector[7]};
            assign quad_0 = {1'b0, pair_0} + {1'b0, pair_1};
            assign quad_1 = {1'b0, pair_2} + {1'b0, pair_3};
            assign total  = {1'b0, quad_0} + {1'b0, quad_1};

            assign lost_this_cycle = {12'd0, total};
        end
        else if (SOURCE_COUNT == 4) begin : g_lost_count_4
            wire [1:0] pair_0;
            wire [1:0] pair_1;
            wire [2:0] total;

            assign pair_0 = {1'b0, lost_vector[0]} +
                            {1'b0, lost_vector[1]};
            assign pair_1 = {1'b0, lost_vector[2]} +
                            {1'b0, lost_vector[3]};
            assign total  = {1'b0, pair_0} + {1'b0, pair_1};

            assign lost_this_cycle = {13'd0, total};
        end
        else begin : g_lost_count_generic
            reg [15:0] count_value;
            integer generic_index;

            always @(*) begin
                count_value = 16'd0;

                for (generic_index = 0;
                     generic_index < SOURCE_COUNT;
                     generic_index = generic_index + 1)
                    count_value =
                        count_value + lost_vector[generic_index];
            end

            assign lost_this_cycle = count_value;
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Pending Slot 갱신
    // -------------------------------------------------------------------------
    // Pending 데이터 배열은 Reset할 필요가 없다. 데이터 쓰기를 비동기
    // Reset 제어 블록과 분리하여 각 Slot이 비동기 Set/Reset 레지스터로
    // 해석되지 않게 한다. pending_valid=0이면 저장값은 외부에 노출되지 않는다.
    always @(posedge clk) begin
        if (!reset_p) begin
            for (data_index = 0;
                 data_index < SOURCE_COUNT;
                 data_index = data_index + 1) begin

                if (event_ready && selected_onehot[data_index]) begin
                    if (source_valid[data_index])
                        pending_data[data_index] <=
                            source_data[(data_index*EVENT_WIDTH)
                                        +: EVENT_WIDTH];
                end
                else if (source_valid[data_index] &&
                         !pending_valid[data_index]) begin

                    pending_data[data_index] <=
                        source_data[(data_index*EVENT_WIDTH)
                                    +: EVENT_WIDTH];
                end
            end
        end
    end

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            pending_valid   <= {SOURCE_COUNT{1'b0}};
            event_lost_pulse <= 1'b0;
            event_lost_count <= 16'd0;

            // pending_valid=0이므로 Pending Data 자체는 Reset하지 않는다.
        end
        else begin
            event_lost_pulse <= 1'b0;

            if (statistics_clear)
                event_lost_count <= 16'd0;

            if (lost_this_cycle != 0) begin
                event_lost_pulse <= 1'b1;

                // 유실 횟수는 최대값에서 포화시킨다.
                if (statistics_clear)
                    event_lost_count <= lost_this_cycle;
                else if (lost_count_sum[16])
                    event_lost_count <= 16'hFFFF;
                else
                    event_lost_count <= lost_count_sum[15:0];
            end

            for (update_index = 0;
                 update_index < SOURCE_COUNT;
                 update_index = update_index + 1) begin

                if (event_ready && selected_onehot[update_index]) begin
                    if (source_valid[update_index]) begin
                        // 기존 Event를 전달한 클럭에 같은 Source의 새 Event가
                        // 들어오면 Slot을 비우지 않고 새 Event로 교체한다.
                        pending_valid[update_index] <= 1'b1;
                    end
                    else begin
                        pending_valid[update_index] <= 1'b0;
                    end
                end
                else if (source_valid[update_index] &&
                         !pending_valid[update_index]) begin

                    pending_valid[update_index] <= 1'b1;
                end
                // Pending Slot이 찬 상태에서 같은 Source Event가 재발생하면
                // 기존 Event를 보존하고 위의 lost 카운터만 증가시킨다.
            end
        end
    end

endmodule
