# Day1~Day6 Read CM Dict - Goto 연결 보고서

작성일: 2026-05-27  
분석 대상: `02_Carmaker_project/Practice_sample/src_cm4sl/Models/Day1` ~ `Day6`

## 1. 분석 범위와 방법

- Simulink 모델 파일 `.slx` 내부의 `simulink/systems/*.xml`을 정적 파싱했다.
- `SourceBlock = CarMaker4SL/Read CM Dict`인 블록을 모두 수집했다.
- 각 Read CM Dict 블록의 `xname` 파라미터와 출력선이 도달하는 `Goto` 블록의 `GotoTag`를 추적했다.
- 백업 모델은 별도 부록으로 분리했다.
- MATLAB 실행 검증은 제외했다. 현재 로컬 MATLAB은 시작 시 `failed to load settings errors_warnings plugin` 오류가 발생했다.

재검토 결과:

- Day1~Day6 폴더의 `.slx` 전체 13개를 다시 스캔했다.
- 활성 모델 10개에는 Read CM Dict가 총 28개 있다.
- 백업 모델까지 포함하면 Read CM Dict가 총 36개 있다.
- 이 보고서 본문은 활성 모델 28개를 정리하고, 백업 모델 8개는 부록에 정리한다.
- `CarMaker4SL/Traffic Object` 블록은 Read CM Dict가 아니므로 본 보고서 범위에서 제외했다.

## 2. 요약

| 구분 | 활성 모델 수 | Read CM Dict 수 | Goto 연결 있음 | Goto 연결 없음 |
|---|---:|---:|---:|---:|
| Day1 | 3 | 3 | 3 | 0 |
| Day2 | 3 | 8 | 8 | 0 |
| Day3 | 2 | 8 | 8 | 0 |
| Day4_5 | 1 | 1 | 0 | 1 |
| Day6 | 1 | 8 | 8 | 0 |
| 합계 | 10 | 28 | 27 | 1 |

주요 특이사항:

- Day2, Day3에는 `Ego_Gloabl_Y` 오타 태그가 존재한다. 표준화 시 `Ego_Global_Y`로 변환해야 한다.
- Day4_5의 Read CM Dict는 `Car.Fr1.tx,Car.Fr1.ty,Car.Fr1.rz,Car.vx`를 한 번에 읽고 Demux를 거쳐 주차 MATLAB Function으로 직접 입력된다. 연결된 Goto는 없다.
- Day6은 ego 상태를 `Ego_V`, `Ego_X`, `Ego_Y`, `Ego_Yaw` 태그로 따로 발행하며, Day2/Day3의 `Ego_Global_X`, `Ego_Gloabl_Y`, `Ego_Vx` 명명 규칙과 다르다.

## 3. Read CM Dict 기준 정리

활성 모델 10개 기준으로 Day 구분 없이 `xname`별 Goto 태그를 정리하면 다음과 같다.

| Read CM Dict `xname` | 연결된 Goto 태그 | 사용 모델/블록 |
|---|---|---|
| `Car.v` | `Ego_Velocity` | `Day1_Scenario_1.slx` / `Read CM Dict2`, `Day1_Scenario_2.slx` / `Read CM Dict3`, `Day2_Scenario_1.slx` / `Read CM Dict3`, `Day2_Scenario_2_3.slx` / `Read CM Dict3` |
| `Car.v` | `Ego_Vx` | `Day2_Scenario_4_5.slx` / `Read CM Dict4`, `Day3_Scenario_1.slx` / `Read CM Dict4`, `Day3_Scenario_2.slx` / `Read CM Dict4` |
| `Car.v` | `Ego_V` | `Day6_Scenario_1.slx` / `Read_Ego_V` |
| `Car.Fr1.tx` | `Ego_Global_X` | `Day2_Scenario_2_3.slx` / `Read CM Dict1`, `Day2_Scenario_4_5.slx` / `Read CM Dict1`, `Day3_Scenario_1.slx` / `Read CM Dict1`, `Day3_Scenario_2.slx` / `Read CM Dict1` |
| `Car.Fr1.tx` | `Ego_X` | `Day6_Scenario_1.slx` / `Read_Ego_X` |
| `Car.Fr1.ty` | `Ego_Gloabl_Y` | `Day2_Scenario_2_3.slx` / `Read CM Dict2`, `Day2_Scenario_4_5.slx` / `Read CM Dict2`, `Day3_Scenario_1.slx` / `Read CM Dict2`, `Day3_Scenario_2.slx` / `Read CM Dict2` |
| `Car.Fr1.ty` | `Ego_Y` | `Day6_Scenario_1.slx` / `Read_Ego_Y` |
| `Car.Fr1.rz` | `Ego_Yaw` | `Day2_Scenario_4_5.slx` / `Read CM Dict3`, `Day3_Scenario_1.slx` / `Read CM Dict3`, `Day3_Scenario_2.slx` / `Read CM Dict3`, `Day6_Scenario_1.slx` / `Read_Ego_Yaw` |
| `Car.Road.Path.DevDist` | `CrossTrackError` | `Day1_Scenario_3_4.slx` / `Read CM Dict`, `Day6_Scenario_1.slx` / `Read CM Dict` |
| `Traffic.T00.rzv` | `Traffic00_YawRate` | `Day6_Scenario_1.slx` / `Read CM Dict1` |
| `Traffic.T01.rzv` | `Traffic01_YawRate` | `Day6_Scenario_1.slx` / `Read CM Dict2` |
| `Traffic.T02.rzv` | `Traffic02_YawRate` | `Day6_Scenario_1.slx` / `Read CM Dict3` |
| `Car.Fr1.tx,Car.Fr1.ty,Car.Fr1.rz,Car.vx` | 없음 | `Day4_5_Scenario_1.slx` / `Read CM Dict1` -> `Demux` -> `MATLAB Function` |

### 같은 Read에 다른 Goto 태그가 있는 항목

| Read CM Dict `xname` | 서로 다른 Goto 태그 | 정리 의견 |
|---|---|---|
| `Car.v` | `Ego_Velocity`, `Ego_Vx`, `Ego_V` | ego speed 표준명 1개로 통합 필요 |
| `Car.Fr1.tx` | `Ego_Global_X`, `Ego_X` | ego global x 표준명 1개로 통합 필요 |
| `Car.Fr1.ty` | `Ego_Gloabl_Y`, `Ego_Y` | `Ego_Gloabl_Y`는 오타. `Ego_Global_Y`로 정규화 권장 |

### 같은 Read에 같은 Goto 태그만 있는 항목

| Read CM Dict `xname` | Goto 태그 |
|---|---|
| `Car.Fr1.rz` | `Ego_Yaw` |
| `Car.Road.Path.DevDist` | `CrossTrackError` |
| `Traffic.T00.rzv` | `Traffic00_YawRate` |
| `Traffic.T01.rzv` | `Traffic01_YawRate` |
| `Traffic.T02.rzv` | `Traffic02_YawRate` |

## 4. 활성 모델 상세

### Day1

| 모델 | Read CM Dict 블록 | SID | CarMaker xname | 연결 Goto 태그 | Goto 블록 | 경로 |
|---|---|---:|---|---|---|---|
| `Day1_Scenario_1.slx` | `Read CM Dict2` | 14 | `Car.v` | `Ego_Velocity` | `Goto1` | Read -> Goto |
| `Day1_Scenario_2.slx` | `Read CM Dict3` | 14 | `Car.v` | `Ego_Velocity` | `Goto1` | Read -> Goto |
| `Day1_Scenario_3_4.slx` | `Read CM Dict` | 4 | `Car.Road.Path.DevDist` | `CrossTrackError` | `Goto1` | Read -> Goto |

### Day2

| 모델 | Read CM Dict 블록 | SID | CarMaker xname | 연결 Goto 태그 | Goto 블록 | 경로 |
|---|---|---:|---|---|---|---|
| `Day2_Scenario_1.slx` | `Read CM Dict3` | 8 | `Car.v` | `Ego_Velocity` | `Goto3` | Read -> Goto |
| `Day2_Scenario_2_3.slx` | `Read CM Dict1` | 28 | `Car.Fr1.tx` | `Ego_Global_X` | `Goto1` | Read -> Goto |
| `Day2_Scenario_2_3.slx` | `Read CM Dict2` | 29 | `Car.Fr1.ty` | `Ego_Gloabl_Y` | `Goto2` | Read -> Goto |
| `Day2_Scenario_2_3.slx` | `Read CM Dict3` | 30 | `Car.v` | `Ego_Velocity` | `Goto3` | Read -> Goto |
| `Day2_Scenario_4_5.slx` | `Read CM Dict1` | 25 | `Car.Fr1.tx` | `Ego_Global_X` | `Goto2` | Read -> Goto |
| `Day2_Scenario_4_5.slx` | `Read CM Dict2` | 26 | `Car.Fr1.ty` | `Ego_Gloabl_Y` | `Goto3` | Read -> Goto |
| `Day2_Scenario_4_5.slx` | `Read CM Dict3` | 27 | `Car.Fr1.rz` | `Ego_Yaw` | `Goto4` | Read -> Goto |
| `Day2_Scenario_4_5.slx` | `Read CM Dict4` | 28 | `Car.v` | `Ego_Vx` | `Goto7` | Read -> Goto |

### Day3

| 모델 | Read CM Dict 블록 | SID | CarMaker xname | 연결 Goto 태그 | Goto 블록 | 경로 |
|---|---|---:|---|---|---|---|
| `Day3_Scenario_1.slx` | `Read CM Dict1` | 69 | `Car.Fr1.tx` | `Ego_Global_X` | `Goto7` | Read -> Goto |
| `Day3_Scenario_1.slx` | `Read CM Dict2` | 70 | `Car.Fr1.ty` | `Ego_Gloabl_Y` | `Goto8` | Read -> Goto |
| `Day3_Scenario_1.slx` | `Read CM Dict3` | 71 | `Car.Fr1.rz` | `Ego_Yaw` | `Goto9` | Read -> Goto |
| `Day3_Scenario_1.slx` | `Read CM Dict4` | 72 | `Car.v` | `Ego_Vx` | `Goto10` | Read -> Goto |
| `Day3_Scenario_2.slx` | `Read CM Dict1` | 39 | `Car.Fr1.tx` | `Ego_Global_X` | `Goto7` | Read -> Goto |
| `Day3_Scenario_2.slx` | `Read CM Dict2` | 40 | `Car.Fr1.ty` | `Ego_Gloabl_Y` | `Goto8` | Read -> Goto |
| `Day3_Scenario_2.slx` | `Read CM Dict3` | 41 | `Car.Fr1.rz` | `Ego_Yaw` | `Goto9` | Read -> Goto |
| `Day3_Scenario_2.slx` | `Read CM Dict4` | 42 | `Car.v` | `Ego_Vx` | `Goto10` | Read -> Goto |

### Day4_5

| 모델 | Read CM Dict 블록 | SID | CarMaker xname | 연결 Goto 태그 | Goto 블록 | 경로 |
|---|---|---:|---|---|---|---|
| `Day4_5_Scenario_1.slx` | `Read CM Dict1` | 3788 | `Car.Fr1.tx,Car.Fr1.ty,Car.Fr1.rz,Car.vx` | 없음 | 없음 | Read -> Demux -> `MATLAB Function` |

Day4_5 세부 연결:

- Read CM Dict 출력은 `Demux` SID 3800으로 들어간다.
- Demux 출력 1~4는 주차 제어 `MATLAB Function` SID 3782의 입력 1~4로 직접 연결된다.
- 이 ego 상태 신호들은 Goto 태그로 발행되지 않는다.

### Day6

| 모델 | Read CM Dict 블록 | SID | CarMaker xname | 연결 Goto 태그 | Goto 블록 | 경로 |
|---|---|---:|---|---|---|---|
| `Day6_Scenario_1.slx` | `Read CM Dict` | 39 | `Car.Road.Path.DevDist` | `CrossTrackError` | `Goto1` | Read -> Goto |
| `Day6_Scenario_1.slx` | `Read CM Dict1` | 40 | `Traffic.T00.rzv` | `Traffic00_YawRate` | `Goto2` | Read -> Goto |
| `Day6_Scenario_1.slx` | `Read CM Dict2` | 41 | `Traffic.T01.rzv` | `Traffic01_YawRate` | `Goto12` | Read -> Goto |
| `Day6_Scenario_1.slx` | `Read CM Dict3` | 42 | `Traffic.T02.rzv` | `Traffic02_YawRate` | `Goto20` | Read -> Goto |
| `Day6_Scenario_1.slx` | `Read_Ego_V` | 60 | `Car.v` | `Ego_V` | `Goto_Ego_V` | Read -> Goto |
| `Day6_Scenario_1.slx` | `Read_Ego_X` | 62 | `Car.Fr1.tx` | `Ego_X` | `Goto_Ego_X` | Read -> Goto |
| `Day6_Scenario_1.slx` | `Read_Ego_Y` | 64 | `Car.Fr1.ty` | `Ego_Y` | `Goto_Ego_Y` | Read -> Goto |
| `Day6_Scenario_1.slx` | `Read_Ego_Yaw` | 66 | `Car.Fr1.rz` | `Ego_Yaw` | `Goto_Ego_Yaw` | Read -> Goto |

## 5. 태그 정규화 관점

| 의미 | 사용 태그 | 등장 위치 | 비고 |
|---|---|---|---|
| Ego 속도 | `Ego_Velocity` | Day1, Day2 Scenario 1~3 | 초기 모델 계열 |
| Ego 속도 | `Ego_Vx` | Day2 Scenario 4~5, Day3 | waypoint/차선 모델 계열 |
| Ego 속도 | `Ego_V` | Day6 | Day6 독자 명명 |
| Ego X | `Ego_Global_X` | Day2 Scenario 2~5, Day3 | 표준에 가까움 |
| Ego X | `Ego_X` | Day6 | Day6 독자 명명 |
| Ego Y | `Ego_Gloabl_Y` | Day2 Scenario 2~5, Day3 | 오타. `Global`이 `Gloabl`로 저장됨 |
| Ego Y | `Ego_Y` | Day6 | Day6 독자 명명 |
| Ego yaw | `Ego_Yaw` | Day2 Scenario 4~5, Day3, Day6 | 공통 사용 |
| Cross-track error | `CrossTrackError` | Day1 Scenario 3~4, Day6 | Road DevDist 기반 |
| Traffic yaw rate | `Traffic00_YawRate`, `Traffic01_YawRate`, `Traffic02_YawRate` | Day6 | `Traffic.T00~T02.rzv` |

통합 시 권장 정규화:

- `Ego_Gloabl_Y` -> `Ego_Global_Y`
- `Ego_Velocity`, `Ego_Vx`, `Ego_V` 중 하나로 표준화
- `Ego_Global_X`와 `Ego_X` 중 하나로 표준화
- `Ego_Global_Y`와 `Ego_Y` 중 하나로 표준화

## 6. 백업 모델 부록

| 모델 | Read CM Dict 블록 | SID | CarMaker xname | 연결 Goto 태그 | Goto 블록 | 경로 |
|---|---|---:|---|---|---|---|
| `Day3_Scenario_2_backup_20260526_094317.slx` | `Read CM Dict1` | 39 | `Car.Fr1.tx` | `Ego_Global_X` | `Goto7` | Read -> Goto |
| `Day3_Scenario_2_backup_20260526_094317.slx` | `Read CM Dict2` | 40 | `Car.Fr1.ty` | `Ego_Gloabl_Y` | `Goto8` | Read -> Goto |
| `Day3_Scenario_2_backup_20260526_094317.slx` | `Read CM Dict3` | 41 | `Car.Fr1.rz` | `Ego_Yaw` | `Goto9` | Read -> Goto |
| `Day3_Scenario_2_backup_20260526_094317.slx` | `Read CM Dict4` | 42 | `Car.v` | `Ego_Vx` | `Goto10` | Read -> Goto |
| `Day4_5_Scenario_1_backup_20260526_094317.slx` | 없음 | - | - | - | - | 백업 모델 내 Read CM Dict 없음 |
| `Day6_Scenario_1_backup_20260526_094317.slx` | `Read CM Dict` | 39 | `Car.Road.Path.DevDist` | `CrossTrackError` | `Goto1` | Read -> Goto |
| `Day6_Scenario_1_backup_20260526_094317.slx` | `Read CM Dict1` | 40 | `Traffic.T00.rzv` | `Traffic00_YawRate` | `Goto2` | Read -> Goto |
| `Day6_Scenario_1_backup_20260526_094317.slx` | `Read CM Dict2` | 41 | `Traffic.T01.rzv` | `Traffic01_YawRate` | `Goto12` | Read -> Goto |
| `Day6_Scenario_1_backup_20260526_094317.slx` | `Read CM Dict3` | 42 | `Traffic.T02.rzv` | `Traffic02_YawRate` | `Goto20` | Read -> Goto |

## 7. 결론

Day1~Day3의 Read CM Dict는 대부분 ego 상태를 읽어서 즉시 Goto로 발행하는 단순 구조다. Day4_5는 예외적으로 ego 상태 4개를 하나의 Read CM Dict에서 읽고 Demux 후 주차 함수에 직접 연결한다. Day6은 ego 상태, cross-track error, traffic yaw-rate를 모두 Goto로 발행하지만 태그 명명 규칙이 이전 Day와 다르다.

통합 모델에서는 InputAdapter 계층에서 모든 Read CM Dict를 모으고, 위 태그들을 표준 신호명으로 재발행하는 방식이 가장 안전하다.
