# Lib_Supervisor Bus Architecture

| 항목 | 내용 |
|---|---|
| **모델** | `Final_Project.slx` / `Lib_Supervisor` |
| **최종 업데이트** | 2026-05-27 (외부 From → Inport 패턴으로 전면 재작성) |
| **백업 파일** | `Final_Project_PreInportRewire_20260527_123714.slx` |
| **구조 패턴** | External From (top-level) → Lib_Supervisor Inport → BusSelector/Direct → BusCreator → Outport |

## 1. 설계 변경 이유 (중요)

Simulink의 **Goto/From 가시성(visibility)** 규칙:
- DNM 영역의 Goto 블록들은 모두 `local` visibility
- `local`은 같은 층위(top level)에서만 보임 → 서브시스템 내부의 From은 못 읽음

따라서 다음 두 가지 옵션이 있었음:
1. DNM Goto의 visibility를 `global`로 바꾸기 → ❌ DNM 수정 금지
2. **External From → Inport 방식** → ✅ 채택

## 2. 데이터 흐름

```
Top Level (Final_Project)                          
  ┌────────────────────────────────────────────┐  
  │  Read CM Dict (13개)                       │  
  │     ↓                                       │  
  │  Goto (13개) [tag: local]                   │  
  │     ↓                                       │  
  │  From (13개, FromSup_*)                     │  
  │     ↓                                       │  
  └─────────┬──────────────────────────────────┘  
            │                                     
            ▼ (13 lines)                         
  ┌────────────────────────────────────────────┐ 
  │ Lib_Supervisor                              │ 
  │   Inport x13                                │ 
  │     ↓                                       │ 
  │   BusSelector x3 (incoming bus 분해)        │ 
  │     ↓                                       │ 
  │   BusCreator x4 (output bus 조립)           │ 
  │     ↓                                       │ 
  │   Outport x4                                │ 
  └─────────┬──────────────────────────────────┘ 
            │                                     
            ▼ (4 buses)                          
     EgoStateBus, MissionBus, EnvironmentBus,     
     SystemBus → 각 제어기                       
```

## 3. 13개 External From → Inport 매핑

| Port | Inport Name | External From | Reads Goto Tag | 분류 |
|---:|---|---|---|---|
| 1  | `In_Ego_Global_Pos`           | `FromSup_EgoGlobalPos`        | `Ego_Global_Pos`           | bus (BSel 분해) |
| 2  | `In_Ego_Velocity`             | `FromSup_EgoVelocity`         | `Ego_Velocity`             | scalar |
| 3  | `In_CrossTrackError`          | `FromSup_CrossTrackError`     | `CrossTrackError`          | scalar (NEW) |
| 4  | `In_Ego_Vx_Body`              | `FromSup_EgoVxBody`           | `Ego_Vx_Body`              | scalar (NEW) |
| 5  | `In_Waypoints`                | `FromSup_Waypoints`           | `waypoints`                | matrix |
| 6  | `In_Parking_Start_Point_XY`   | `FromSup_ParkingStartXY`      | `Parking_Start_Point_XY`   | bus (BSel 분해) |
| 7  | `In_Parking_Goal_Point`       | `FromSup_ParkingGoalPoint`    | `Parking_Goal_Point`       | bus (BSel 분해) |
| 8  | `In_Parking_Map_Boundary`     | `FromSup_ParkingMapBoundary`  | `Parking_Map_Boundary`     | array |
| 9  | `In_Obstacle_Info`            | `FromSup_ObstacleInfo`        | `Obstacle_Info`            | vector |
| 10 | `In_Traffic00_YawRate`        | `FromSup_T00YawRate`          | `Traffic00_YawRate`        | scalar (NEW) |
| 11 | `In_Traffic01_YawRate`        | `FromSup_T01YawRate`          | `Traffic01_YawRate`        | scalar (NEW) |
| 12 | `In_Traffic02_YawRate`        | `FromSup_T02YawRate`          | `Traffic02_YawRate`        | scalar (NEW) |
| 13 | `In_Simulation_Time`          | `FromSup_SimulationTime`      | `Simulation_Time`          | scalar |

## 4. 4개 출력 버스 구성

### 4.1 EgoStateBus (6 신호)
| 신호명 | 출처 |
|---|---|
| `Car_Fr1_tx` | In_Ego_Global_Pos → BSel_Ego |
| `Car_Fr1_ty` | In_Ego_Global_Pos → BSel_Ego |
| `Car_Fr1_rz` | In_Ego_Global_Pos → BSel_Ego |
| `Ego_V` | In_Ego_Velocity |
| `CrossTrackError` ⭐ | In_CrossTrackError |
| `Ego_Vx_Body` ⭐ | In_Ego_Vx_Body |

### 4.2 MissionBus (6 신호)
| 신호명 | 출처 |
|---|---|
| `Waypoints` | In_Waypoints |
| `Parking_Start_Point_X/Y` | In_Parking_Start_Point_XY → BSel_ParkingStart |
| `Parking_Goal_Point_X/Y/Yaw` | In_Parking_Goal_Point → BSel_ParkingGoal |

### 4.3 EnvironmentBus (5 신호)
| 신호명 | 출처 |
|---|---|
| `MapBoundary` | In_Parking_Map_Boundary |
| `ObstacleInfo` | In_Obstacle_Info |
| `Traffic00_YawRate` ⭐ | In_Traffic00_YawRate |
| `Traffic01_YawRate` ⭐ | In_Traffic01_YawRate |
| `Traffic02_YawRate` ⭐ | In_Traffic02_YawRate |

### 4.4 SystemBus (1 신호)
| 신호명 | 출처 |
|---|---|
| `SimTime` | In_Simulation_Time |

## 5. Lib_Supervisor 내부 블록 (총 24개)

| 종류 | 개수 |
|---|---:|
| Inport | 13 |
| BusSelector | 3 |
| BusCreator | 4 |
| Outport | 4 |
| **합계** | **24** |

> 내부에 From/Goto 블록 없음 (모든 신호는 Inport로 들어옴)

## 6. 변경 이력

| 일자 | 변경 |
|---|---|
| 2026-05-27 (1) | 기존 9-input/1-output → 0-input/4-output 버스 구조 (내부 From 사용) |
| 2026-05-27 (2) | Day1~6 신호 5개 추가 |
| 2026-05-27 (3) | **Goto visibility 문제로 인해 External From → Inport 패턴으로 재작성** (13 input/4 output) |
