# 협업 기록

원본 팀 저장소 [`ChungRyeung/26HL_IVS_ADAS`](https://github.com/ChungRyeung/26HL_IVS_ADAS)는 **private**이므로, 이 문서에 협업 구조와 PR 이력을 기록으로 남깁니다. `team-project/`의 커밋 히스토리는 원본 저자 표기 그대로 미러되어 있습니다 (`git log -- team-project`로 확인 가능).

## 팀 구성 및 역할 (6인 + 멘토)

| GitHub | 역할 | 담당 모듈 |
|---|---|---|
| [@ChungRyeung](https://github.com/ChungRyeung) | 팀원 | Supervisor · Mission Manager FSM · 모델 통합 및 형상관리 |
| [@POOH0119](https://github.com/POOH0119) | 팀원 | 주행 알고리즘 (`Lib_Driving`) — 차선 유지 · 전방 추월 판단 |
| [@ParkJinSui](https://github.com/ParkJinSui) | 팀원 | 주행 컨트롤러 (`VC_Driving`) — 모드별 PID · 속도 적응 |
| [@preference-park98](https://github.com/preference-park98) | 팀원 | 톨게이트 알고리즘 (`Lib_Tollgate`) — 하이패스 진입 · 단계 감속 |
| **[@jb-cho55](https://github.com/jb-cho55)** | **팀장** | **주차 알고리즘 (`Lib_Parking`) — Hybrid A* + Staging 전략** |
| [@hackisha](https://github.com/hackisha) | 팀원 | 주차 컨트롤러 (`VC_Parking`) — 저속 정밀 추종 제어 |

- 주차 파트(Lib_Parking + VC_Parking)는 @jb-cho55 · @hackisha **페어 프로그래밍**으로 진행 — 팀 저장소의 주차 커밋은 페어 세션 환경에서 @hackisha 계정으로 푸시됨.
- 멘토(송승목): 계획 검토 및 프로젝트 멘토링.

## 개발 프로세스 (2주)

| 구분 | 기간 | 활동 |
|---|---|---|
| 사전 기획 | 05.21~05.22 | 통합 시나리오 설계 · 1인 1모듈 분담 결정 |
| 모델 분석 | 05.22~05.23 | CarMaker 연동 · 입력 13신호 정의 · Bus 18개 설계 (인터페이스 합의) |
| 알고리즘 구현 | 05.23~05.28 | 주행/톨게이트/주차 알고리즘 `.m` 함수로 분리 구현 |
| 통합 | 05.28~06.03 | Supervisor · Mission Manager FSM · Switch 배선 (폐루프 구성) |
| 개선·검증 | 06.03~06.05 | Staging 전략 적용 · CarMaker 폐루프 시험 |

## 브랜치 전략

- **이슈 기반 브랜치**: `#12_driving_module_waypoint_problem`, `#12-fix_waypoint_problem-Sunho`, `#23-refactor_remove_mission_controller-Sunho` 등 이슈 번호를 브랜치명에 연결
- **기능/개인 브랜치**: `parking`, `making_parking_set`, `Driving_Module_Ver.2_PSH`, `0602_YOO` 등
- **통합 브랜치**: `parking_integration`, `Tollgate_integration`, `merging_all_resources` — 모듈별 작업을 단계적으로 main에 병합
- main은 PR 머지로만 갱신 (총 브랜치 26개)

## PR 타임라인 (전체 22건)

| # | 상태 | 작성자 | 브랜치 | 제목 |
|---|---|---|---|---|
| #26 | CLOSED | @hackisha | codex/tollgate-lane1-progress-20260606 | Checkpoint tollgate lane1 driving progress |
| #25 | OPEN | @POOH0119 | #23-refactor_remove_mission_controller-Sunho | refactor: remove mission controller from driving module |
| #24 | MERGED | @hackisha | parking_integration | Parking integration |
| #22 | CLOSED | @ChungRyeung | parking_integration | Parking integration |
| #20 | MERGED | @preference-park98 | #12-fix_waypoint_problem-Sunho | fix: add waypoints for driving module merge issue |
| #19 | MERGED | @POOH0119 | 0602_YOO | 통합 |
| #18 | MERGED | @ChungRyeung | making_parking_set | finished parking |
| #17 | MERGED | @ChungRyeung | making_parking_set | changed logic of switch |
| #16 | MERGED | @ChungRyeung | making_parking_set | Making parking set |
| #15 | MERGED | @ChungRyeung | #12_driving_module_waypoint_problem | added tollgate state |
| #14 | MERGED | @ChungRyeung | #12_driving_module_waypoint_problem | fixed logic of module swith |
| #13 | MERGED | @ChungRyeung | #12_driving_module_waypoint_problem | change solved |
| #11 | MERGED | @ChungRyeung | merging_all_sequence | Merging all sequence |
| #10 | MERGED | @hackisha | parking | chore: 빌드 아티팩트·개인 작업파일 git 추적 해제 (로컬 보존) |
| #9 | CLOSED | @hackisha | cleanup/untrack-artifacts | chore: 빌드 아티팩트·개인 작업파일 git 추적 해제 |
| #8 | MERGED | @hackisha | parking | feat: 플래너+검증 종방향제어 통합 — 전·후진 주차 완주 ✨ |
| #7 | MERGED | @hackisha | parking | feat: RRT*+CCRS 후진주차 플래너 + 진행상황 (Day4_5) ✨ |
| #6 | MERGED | @ParkJinSui | day6_pjs_solved | Day6 pjs solved |
| #5 | MERGED | @ChungRyeung | #01_bus-setting&supervisor-setting | #01 bus setting&supervisor setting |
| #3 | MERGED | @preference-park98 | test | 20260527_학습용 |
| #2 | MERGED | @preference-park98 | test | Day1_2 |

> PR #7 · #8 · #24 (주차 플래너 → 종방향제어 통합 → 최종 주차 통합)가 주차 파트 페어 작업의 결과물입니다.

## 협업에서 배운 것

- **인터페이스 먼저 합의**: 개발 초기에 입력 13신호 · Bus 18개를 먼저 확정 → 모듈 간 병렬 개발에서 머지 충돌 최소화
- **1인 1모듈 + `.m` 분리**: Simulink 모델(.mdl) 충돌을 피하기 위해 알고리즘을 MATLAB 함수로 분리, 모델 배선은 형상관리 담당 1인이 전담
- **통합 브랜치 단계 병합**: 개별 기능 → `*_integration` → main 순서로 병합해 통합 리스크를 격리
