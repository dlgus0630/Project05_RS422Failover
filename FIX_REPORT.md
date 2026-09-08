# Final RTL Review and Fix Report

## 검토 범위

- 작업 브랜치: `fix/final-rtl-review`
- 기준 커밋: `4f0991309c936dd4068d18500e1eabf13c37ca73`
- 대상 FPGA/도구: `xc7a35tcpg236-1`, Vivado 2024.2
- 문법 기준: Verilog-2005 (`xvlog`를 SystemVerilog 옵션 없이 실행)
- 저장소 루트에는 ZIP 원본이 없었으며, 이미 압축 해제되어 있던
  `redundant_link.srcs/redundant_link_rtl_final/`을 읽기 전용 참조본으로
  사용했다. 이 디렉터리와 모든 ZIP은 `.gitignore`로 제외했다.

## 최종 인터페이스 반영

다음 8개 RTL을 참조본의 최종 인터페이스 기준으로 교체한 뒤, 아래의
기능 및 타이밍 수정 사항을 반영했다.

- `frame_parser.v`
- `pair_matcher.v`
- `channel_health_mgr.v`
- `decision_unit.v`
- `duplicate_guard.v`
- `raw_frame_buffer.v`
- `event_arbiter.v`
- `event_fifo.v`

`axi_lite_regs.v`는 기준 커밋과 바이트 단위로 동일하며, AXI 주소 맵도
변경하지 않았다. `redundant_link_core.v`는 저장소 버전을 기준으로
최종 인터페이스를 연결했다. 프로토콜, 프레임 형식, 모듈명 및 XDC는
변경하지 않았다.

## 수정 내용

### Raw frame backpressure

- FIFO가 가득 찬 동안 producer가 `in_valid`와 데이터를 유지하는 정상
  backpressure를 overflow로 계산하지 않도록 수정했다.
- 대기 중인 요청이 handshake 전에 철회된 경우에만 overflow pulse와
  count가 정확히 한 번 증가한다.
- Core의 translation 출력은 `ready`가 들어올 때까지 valid와 데이터를
  유지한다.

### FND 선택 채널 표시

- 한 프레임 전의 등록값 대신 현재 전송 handshake의
  `translation_selected_b`를 status display에 전달한다.
- Core TB에서 B 채널 선택과 sequence `8'h13`이 같은 프레임에 표시되는
  것을 검사한다.

### Reset/Clear 구조

- 비동기 sensitivity list에는 `reset_p`만 남겼다.
- `system_enable`, channel timeout, FIFO clear 및 각 datapath clear는
  clocked block 내부의 동기식 clear로 처리한다.
- `uart_rx`, `uart_tx`, `crc16_ccitt`, `frame_parser`, `pair_matcher`,
  `channel_health_mgr`, `duplicate_guard`, `raw_frame_buffer`,
  `seq_monitor`, `frame_fifo`에 필요한 동기 clear 경로를 연결했다.
- timeout에 의한 sequence monitor/FIFO 초기화는 같은 상승 에지에서
  적용되도록 timeout fire를 사용한다.

### 합성 및 타이밍 안정화

- `raw_frame_buffer`, `event_fifo`, `event_arbiter`, `frame_fifo`의 데이터
  배열 쓰기를 비동기 reset 제어 블록과 분리해 distributed RAM으로
  추론되도록 했다. valid/count가 0이면 이전 데이터는 관찰되지 않는다.
- `event_arbiter`의 16-source 유실 개수를 균형형 popcount tree로
  계산한다.
- 최우선 pending source는 one-hot으로 분리하고, 유실 count 포화 연산은
  단일 17-bit 확장 덧셈으로 계산해 같은 cycle 동작을 유지하면서
  100 MHz 임계 경로를 줄였다.
- 채널 복구 판정은 현재 valid/equal pair와 local fail 상태를 직접
  사용하도록 정리했다.

## 시뮬레이션 결과

전체 Design Sources와 Simulation Sources를 깨끗한 임시 work library에서
다시 컴파일했다. 모든 TB는 최종 RTL 상태로 elaboration 및 simulation을
통과했다.

| Testbench | Elaboration | Simulation |
|---|---:|---:|
| `tb_axi_lite_regs` | PASS | PASS |
| `tb_channel_health_mgr` | PASS | PASS |
| `tb_crc16_ccitt` | PASS | PASS |
| `tb_decision_unit` | PASS | PASS |
| `tb_duplicate_guard` | PASS | PASS |
| `tb_event_arbiter` | PASS | PASS |
| `tb_event_fifo` | PASS | PASS |
| `tb_frame_fifo` | PASS | PASS |
| `tb_frame_parser` | PASS | PASS |
| `tb_pair_matcher` | PASS | PASS |
| `tb_raw_frame_buffer` | PASS | PASS |
| `tb_seq_monitor` | PASS | PASS |
| `tb_status_display` | PASS | PASS |
| `tb_uart_rx` | PASS | PASS |
| `tb_uart_tx` | PASS | PASS |
| `tb_redundant_link_core` | PASS | PASS |

- `xvlog`: PASS, compile error/warning 없음
- `xelab`: 16/16 PASS, port 및 width 불일치 없음
- `xsim`: 16/16 PASS, TB fail 없음

## 합성, 타이밍 및 DRC

`redundant_link_core`를 100 MHz clock으로 out-of-context 합성, 배치,
물리 최적화 및 라우팅했다.

- 합성: error 0, critical warning 0
- 라우팅: failed/unrouted/partially routed net 0
- Setup: WNS `+0.143 ns`, TNS `0.000 ns`, failing endpoint 0
- Hold: WHS `+0.028 ns`, THS `0.000 ns`, failing endpoint 0
- DRC: error 0
- Latch: latch inference 메시지 없음, combinational latch loop 0
- 다중 드라이버: 검출 없음
- 포트/폭 불일치: `xvlog`, `xelab`, 합성에서 검출 없음

## 남은 경고

아래 경고는 검증 실패나 RTL 연결 오류가 아니다.

- 변경하지 않은 `axi_lite_regs.v`의 32-bit merged register 중 실제로
  사용하지 않는 상위 bit가 잘리는 `Synth 8-3936` 경고 8건
- OOC top에 board configuration property가 없어 발생한 `CFGBVS-1`
- OOC output port가 parent block design에 연결되지 않아 발생한
  `RTSTAT-10`
- parent design의 pin/clock 위치 및 I/O delay가 OOC 실행에 없어서
  발생한 `HD.CLK_SRC`, `HD.PARTPIN_LOCS`, `TIMING-18` 경고
- 실행 환경의 local Tcl store 쓰기 권한 경고와 parallel synthesis
  기준 미충족 경고

위 OOC 제한에도 내부 100 MHz register-to-register setup/hold 제약은
모두 만족했다. 실제 bitstream sign-off에서는 parent block design과
board XDC를 포함한 전체 프로젝트 implementation 결과를 최종 기준으로
사용해야 한다.

## 커밋 제외 확인

- ZIP 파일 없음
- `redundant_link_rtl_final/` 참조 디렉터리 ignore 확인
- `.runs`, `.cache`, `.sim`, `.Xil`, `xsim.dir` 미포함
- Vivado 임시 산출물은 `/tmp`에서만 생성했으며 커밋하지 않음
