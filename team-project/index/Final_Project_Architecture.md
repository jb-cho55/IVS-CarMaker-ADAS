# Final_Project Architecture
| 항목 | 내용 |
|---|---|
| **모델** | `Final_Project.slx` |
| **최종 업데이트** | 2026-05-27 (시나리오 기반 7-Lib 구조 + chart trigger 직결) |
| **백업 파일** | `Final_Project_PreSelectorCtrl_20260527_144759.slx` 외 다수 |
| **구조 패턴** | 시나리오 모듈(3) + 차량 컨트롤러(2) + 미션 매니저(1) + 슈퍼바이저(1) |

## 1. 설계 배경

기존 구조는 **역할 단위**(횡제어/종제어/모드/경로/궤적)로 모듈을 나눠 한 시나리오를 처리하려면 여러 Lib를 동시에 건드려야 했음. 시나리오 단위로 재편하여 각 모듈의 책임을 명확히 함.

- **주행 시나리오** = 일반 주행 + 추월
- **톨게이트 시나리오** = 하이패스 차선 진입 + 통과
- **주차 시나리오** = A* 경로 + 주차 maneuver (전·후진 모두)

## 2. 데이터 흐름

```
                    Read CM Dict (13개)
                          ↓
                       Goto (local)
                          ↓
                    FromSup_* (13개)
                          ↓
              ┌──────────────────────┐
              │   Lib_Supervisor     │
              │   13 in → 4 buses    │
              └──────────┬───────────┘
                         ↓
         ┌──────────┬────┴─────┬──────────────┐
         ▼          ▼          ▼              ▼
      EgoState   Mission   Environment    System (예약)
         │          │          │
         └──┬──┬──┬─┴──┬──┬──┬─┘
            ▼  ▼  ▼    ▼  ▼  ▼
         ┌──────────────────────┐
         │ Lib_MissionManager   │  ── ActiveScenario, SubMode×3
         └──────────────────────┘
                    ↓                            
   ┌────────────┬─────────────┬─────────────┐
   ▼            ▼             ▼             
 Lib_Driving  Lib_Tollgate  Lib_Parking     
 (cruise+OT)  (hi-pass)     (A* + park)     
   │ ×4 out     │ ×4 out      │ ×4 out      
   ▼            ▼             ▼             
  TrajBus, TargetSpeed, Status, SelectorCtrl 
   │            │             │              
   └────┬───────┘             │              
        ▼                     ▼              
  ┌──────────────────┐  ┌──────────────────┐
  │ VC_Driving (10ms)│  │ VC_Parking (50ms)│
  │ Stanley/PP + PID │  │ low-speed Pure PP│
  └────────┬─────────┘  └────────┬─────────┘
           │                     │           
        SteerCmd_Driving       SteerCmd_Parking
        AccelCmd_Driving       AccelCmd_Parking
           │                     │           
           └──────┬──────────────┘           
                  ▼                          
           Switch_Steer/Accel (by ActiveScenario>2.5)
                  ↓                          
           Sat_Steer/Accel                   
                  ↓                          
           Write_SteerL/R, Write_AccelCmd    

   (별도) DrivingSelectorCtrl + TollgateSelectorCtrl + ParkingSelectorCtrl
                  ↓                          
           Switch_SelectorCtrl (MultiPort, by ActiveScenario index)
                  ↓                          
           Write CM Dict4 (DM.SelectorCtrl) 
```

## 3. 7개 Lib 모듈

| Lib | Dev | 주기 | 역할 |
|---|:-:|:-:|---|
| `Lib_Supervisor` | F | - | 13 input → 4 bus 분배 (EgoState/Mission/Environment/System) |
| `Lib_MissionManager` | F | 10ms | DRIVING ↔ TOLLGATE ↔ PARKING FSM, sub-mode 발행 |
| `Lib_Driving` | A | 10ms | 일반 주행 + 추월 (cruise + overtake) |
| `Lib_Tollgate` | B | 100ms | 하이패스 차선 진입 + 통과 |
| `Lib_Parking` | D | 50ms | A* 경로 + 주차 maneuver (전·후진) |
| `Lib_VehicleController_Driving` | C | 10ms | 주행·톨게이트 trajectory tracking (Stanley/PP + PID) |
| `Lib_VehicleController_Parking` | E | 50ms | 저속 정밀 tracking (전후진 인식) |

## 4. 각 Lib 인터페이스

### 4.1 Lib_Supervisor (13 in / 4 out / no trigger)

| Port | Inport | 출처 (FromSup) | Reads Goto |
|---:|---|---|---|
| 1 | `In_Ego_Global_Pos` | `FromSup_EgoGlobalPos` | `Ego_Global_Pos` (bus) |
| 2 | `In_Ego_Velocity` | `FromSup_EgoVelocity` | `Ego_Velocity` |
| 3 | `In_CrossTrackError` | `FromSup_CrossTrackError` | `CrossTrackError` |
| 4 | `In_Ego_Vx_Body` | `FromSup_EgoVxBody` | `Ego_Vx_Body` |
| 5 | `In_Waypoints` | `FromSup_Waypoints` | `waypoints` |
| 6 | `In_Parking_Start_Point_XY` | `FromSup_ParkingStartXY` | `Parking_Start_Point_XY` |
| 7 | `In_Parking_Goal_Point` | `FromSup_ParkingGoalPoint` | `Parking_Goal_Point` |
| 8 | `In_Parking_Map_Boundary` | `FromSup_ParkingMapBoundary` | `Parking_Map_Boundary` |
| 9 | `In_Obstacle_Info` | `FromSup_ObstacleInfo` | `Obstacle_Info` |
| 10 | `In_Traffic00_YawRate` | `FromSup_T00YawRate` | `Traffic00_YawRate` |
| 11 | `In_Traffic01_YawRate` | `FromSup_T01YawRate` | `Traffic01_YawRate` |
| 12 | `In_Traffic02_YawRate` | `FromSup_T02YawRate` | `Traffic02_YawRate` |
| 13 | `In_Simulation_Time` | `FromSup_SimulationTime` | `Simulation_Time` |

**출력 버스 4개**: `EgoStateBus` / `MissionBus` / `EnvironmentBus` / `SystemBus`

### 4.2 Lib_MissionManager (5 in / 4 out / trigger)

| Inports | Outports |
|---|---|
| `In_EgoStateBus` | `Out_ActiveScenario` (int32, 1=DRIVING/2=TOLLGATE/3=PARKING) |
| `In_MissionBus` | `Out_DrivingSubMode` (int32) |
| `In_DrivingStatus` | `Out_TollgateSubMode` (int32) |
| `In_TollgateStatus` | `Out_ParkingSubMode` (int32) |
| `In_ParkingStatus` | |

### 4.3 Lib_Driving / Lib_Tollgate / Lib_Parking (4 in / 4 out / trigger, 공통 인터페이스)

| Inports | Outports |
|---|---|
| `In_EgoStateBus` | `Out_TrajectoryBus` (100×3 [x y yaw]) |
| `In_MissionBus` | `Out_TargetSpeed` (double, m/s) |
| `In_EnvironmentBus` | `Out_<Scenario>Status` (int32) |
| `In_<Scenario>SubMode` | `Out_SelectorCtrl` (int32, gear) |

**SelectorCtrl 의미** (`DM.SelectorCtrl`):
- `1` = D (Drive) — Driving / Tollgate 항상
- `-1` = R (Reverse) — Parking 후진 시
- `0` = N — Parking 전환 중
- `-9` = P — Parking 완료

### 4.4 Lib_VehicleController_Driving (6 in / 2 out / trigger)

| Inports | Outports |
|---|---|
| `In_EgoStateBus` | `Out_SteerCmd` (rad) |
| `In_ActiveScenario` (control: 1=Driving, 2=Tollgate) | `Out_AccelCmd` (m/s²) |
| `In_DrivingTrajectoryBus` | |
| `In_DrivingTargetSpeed` | |
| `In_TollgateTrajectoryBus` | |
| `In_TollgateTargetSpeed` | |

### 4.5 Lib_VehicleController_Parking (3 in / 2 out / trigger)

| Inports | Outports |
|---|---|
| `In_EgoStateBus` | `Out_SteerCmd` (rad) |
| `In_ParkingTrajectoryBus` | `Out_AccelCmd` (m/s², 부호로 전후진) |
| `In_ParkingTargetSpeed` | |

## 5. 트리거 (스케줄링)

Stateflow chart가 생성하는 `do_Logic_<rate>_<idx>` function-call 신호를 각 Lib가 **직접 구독**.

| Lib | Trigger 입력 | 주기 |
|---|---|:-:|
| `Lib_MissionManager` | `From[do_Logic_10ms_1]` | 10ms |
| `Lib_Driving` | `From[do_Logic_10ms_1]` | 10ms |
| `Lib_VehicleController_Driving` | `From[do_Logic_10ms_2]` | 10ms |
| `Lib_Tollgate` | `From[do_Logic_100ms_1]` | 100ms |
| `Lib_Parking` | `From[do_Logic_50ms_1]` | 50ms |
| `Lib_VehicleController_Parking` | `From[do_Logic_50ms_2]` | 50ms |

**미사용 슬롯**: `do_Logic_100ms_2`, `do_Logic_50ms_2`의 일부 채널은 예약.

> ⚠ Chart 자체는 **수정하지 않음**. Layer 0(Stateflow)의 출력 태그를 그대로 사용.

## 6. 출력 체인 (Lib → Write CM Dict)

### 6.1 Steer 및 Accel

```
VC_Driving/SteerCmd  ──→ Goto[SteerCmd_Driving]  ──┐
                                                    ├──→ Switch_Steer ──→ Sat_Steer ──┬──→ Write_SteerL (Car.CFL.rz_ext)
VC_Parking/SteerCmd  ──→ Goto[SteerCmd_Parking]  ──┘     (criteria:                  └──→ Write_SteerR (Car.CFR.rz_ext)
                                                          u2 > 2.5 → Parking)
From[ActiveScenario] ──────────────────────────→ control (u2)
```

(Accel도 동일 패턴, Sat_Accel → Write_AccelCmd, `AccelCtrl.DesiredAx`)

### 6.2 SelectorCtrl (시나리오에서 직접 전달)

```
Lib_Driving/Out_SelectorCtrl   ──→ Goto[DrivingSelectorCtrl]   ──→ MultiPort idx 1
Lib_Tollgate/Out_SelectorCtrl  ──→ Goto[TollgateSelectorCtrl]  ──→ MultiPort idx 2
Lib_Parking/Out_SelectorCtrl   ──→ Goto[ParkingSelectorCtrl]   ──→ MultiPort idx 3
From[ActiveScenario] ─────────────────────────────────────────→ control
                                                                       ↓
                                                          Switch_SelectorCtrl (MultiPortSwitch, indices {1,2,3})
                                                                       ↓
                                                          Write CM Dict4 (DM.SelectorCtrl)
```

### 6.3 Saturation 한계값
| 신호 | 하한 | 상한 |
|---|:-:|:-:|
| Steer | -π/6 | +π/6 |
| Accel | -5.0 | +3.0 |

## 7. CarMaker Read CM Dict (Input)

### 7.1 DNM 영역 (SID 58~62, 절대 수정 금지)
| SID | 블록 | xname |
|:-:|---|---|
| 58 | `Read CM Dict1` | `Car.Fr1.tx` |
| 59 | `Read CM Dict2` | `Car.Fr1.ty` |
| 60 | `Read CM Dict3` | `Car.Fr1.rz` |
| 61 | `Read CM Dict4` | `Time` → `Goto[Simulation_Time]` |
| 62 | `Read CM Dict5` | `Car.v` → `Goto[Ego_Velocity]` |

### 7.2 Day1~6 신호 (Modify 영역)
| SID | 블록 | xname | Goto |
|:-:|---|---|---|
| 3927 | `Read_DevDist_Lib` | `Car.Road.Path.DevDist` | `CrossTrackError` |
| 3935 | `Read_EgoVxBody_Lib` | `Car.vx` | `Ego_Vx_Body` |
| 3929 | `Read_T00_YawRate_Lib` | `Traffic.T00.rzv` | `Traffic00_YawRate` |
| 3931 | `Read_T01_YawRate_Lib` | `Traffic.T01.rzv` | `Traffic01_YawRate` |
| 3933 | `Read_T02_YawRate_Lib` | `Traffic.T02.rzv` | `Traffic02_YawRate` |

> ⚠ **Traffic ID 검증 필요**: `99_Integration_Plan_v2_PATCH.md` 기준 Final_Project 시나리오는 `Traffic.T22~T28`을 사용. 현재 T00~T02은 `day7_final` 시나리오 확인 후 정정 필요.

## 8. CarMaker Write CM Dict (Output)

| 블록 | xname | 입력 체인 |
|---|---|---|
| `Write_SteerL` | `Car.CFL.rz_ext` | Sat_Steer ← Switch_Steer |
| `Write_SteerR` | `Car.CFR.rz_ext` | Sat_Steer ← Switch_Steer |
| `Write_AccelCmd` | `AccelCtrl.DesiredAx` | Sat_Accel ← Switch_Accel |
| `Write CM Dict4` | `DM.SelectorCtrl` | Switch_SelectorCtrl (시나리오 발행) |

## 9. functions/ 폴더 (외부 wrapper .m)

각 Lib의 MATLAB Function 블록은 외부 .m 파일을 wrapper로 호출. 알고리즘 수정은 .m 파일에서.

| 파일 | Lib | Dev | 상태 |
|---|---|:-:|:-:|
| `mission_manager_fcn.m` | Lib_MissionManager | F | placeholder |
| `driving_scenario_fcn.m` | Lib_Driving | A | placeholder |
| `tollgate_scenario_fcn.m` | Lib_Tollgate | B | placeholder |
| `parking_scenario_fcn.m` | Lib_Parking | D | placeholder |
| `vehicle_controller_driving_fcn.m` | Lib_VehicleController_Driving | C | placeholder |
| `vehicle_controller_parking_fcn.m` | Lib_VehicleController_Parking | E | placeholder |

**구 함수 (참고용 보존)**: `dev_a_basic_driving.m`, `dev_b_overtaking.m`, `dev_c_tollgate.m`, `dev_d_parking.m`, `dev_e_lateral.m`, `dev_f_supervisor.m`

## 10. Top-level Goto 태그 통계

| 분류 | 개수 |
|---|---:|
| Supervisor 입력 (Read CM Dict + BusCreator) | 13 |
| Supervisor 출력 4 buses | 4 |
| MissionManager 출력 (ActiveScenario + 3 SubMode) | 4 |
| 시나리오 출력 (3개 × 4 신호) | 12 |
| VehicleController 출력 (2개 × 2 신호) | 4 |
| Traffic_Info 옛 신호 (Day6 잔재) | 28 |
| Scheduler (do_Logic_*) | 6 |
| **합계 (top-level)** | **약 71** |

## 11. Dev 분담 (시나리오 기반)

| Dev | 담당 Lib | 담당 함수 |
|:-:|---|---|
| A | Lib_Driving | `driving_scenario_fcn.m` |
| B | Lib_Tollgate | `tollgate_scenario_fcn.m` |
| C | Lib_VehicleController_Driving | `vehicle_controller_driving_fcn.m` |
| D | Lib_Parking | `parking_scenario_fcn.m` |
| E | Lib_VehicleController_Parking | `vehicle_controller_parking_fcn.m` |
| F | Lib_Supervisor + Lib_MissionManager + 통합 | `mission_manager_fcn.m` + 상위 wiring |

## 12. 변경 이력

| 일자 | 변경 |
|---|---|
| 2026-05-27 (1) | 역할 기반 5-Lib 구조 → **시나리오 기반 8-Lib 구조** (Supervisor + MM + 3 시나리오 + 2 VC + OutputAdapter) |
| 2026-05-27 (2) | 1:1 task-specific trigger 매핑 (`trig_<Task>_<Rate>ms`) 도입 — 추후 chart 직결로 폐기 |
| 2026-05-27 (3) | dead block 정리 (옛 Ext_Lib_* Constants, 옛 SelectorCtrl 체인 등 15개) |
| 2026-05-27 (4) | `Lib_OutputAdapter` 제거 → 톱레벨 Switch_Steer/Accel 으로 단순화 (7-Lib 구조) |
| 2026-05-27 (5) | `SelectorCtrl`을 시나리오 단위로 전달하는 형식으로 변경 (3 Goto + MultiPortSwitch) |
| 2026-05-27 (6) | Trigger를 chart의 `do_Logic_*ms_*` **직결 구독**으로 전환 (SchedFCS 체인 제거) |

## 13. 미해결 항목

1. **🔴 Traffic ID 정정** — T00~T02 → T22~T28 (`day7_final` 시나리오 검증)
2. **🔴 DVA 준수** — Read CM Dict 13개를 `00_InputAdapter` 서브시스템으로 패키징
3. **🟡 MissionManager FSM 실제 구현** (Dev-F)
4. **🟡 각 시나리오 알고리즘 구현** (Dev-A/B/D placeholder → 실제 로직)
5. **🟡 Vehicle Controller 알고리즘 구현** (Dev-C/E placeholder → 실제 Stanley/PP + PID)
