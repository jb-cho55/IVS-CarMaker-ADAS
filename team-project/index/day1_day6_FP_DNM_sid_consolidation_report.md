# Day1~Day6 + Final Project DNM 통합 SID 정리 보고서

| 항목 | 내용 |
|---|---|
| **작성일** | 2026-05-27 |
| **대상 모델** | `Day1_Scenario_1`~`Day6_Scenario_1` (10개) + `Final_Project.slx` |
| **분석 범위** | 활성 모델만 (백업 제외). Final Project는 "Do Not Modify" 영역만 |
| **충돌 우선순위** | Final Project DNM > Day1~Day6 |
| **방법** | MATLAB 런타임에서 `find_system` + `get_param` 직접 조회 |


## 📑 목차

1. [분석 범위](#1-분석-범위)
2. [요약](#2-요약)
3. [SID 충돌 요약 (Final Project DNM 우선)](#3-sid-충돌-요약-final-project-dnm-우선)
4. [Final Project "Do Not Modify" 영역 상세](#4-final-project-do-not-modify-영역-상세)
5. [Read CM Dict 통합 표 (SID 순)](#5-read-cm-dict-통합-표-sid-순)
6. [Goto 통합 표](#6-goto-통합-표)
7. [통합 모델 마이그레이션 가이드](#7-통합-모델-마이그레이션-가이드)
8. [검증](#8-검증)
9. [관련 보고서](#9-관련-보고서)

## 1. 분석 범위

### 1.1 Day1~Day6 모델 (10개)
- `Day1_Scenario_1`, `Day1_Scenario_2`, `Day1_Scenario_3_4`
- `Day2_Scenario_1`, `Day2_Scenario_2_3`, `Day2_Scenario_4_5`
- `Day3_Scenario_1`, `Day3_Scenario_2`
- `Day4_5_Scenario_1`
- `Day6_Scenario_1`

### 1.2 Final_Project.slx — "Do Not Modify" 영역
annotation 위치: `[x: 1975~2624, y: 1304~2533]` (모델 좌측 영역)

해당 영역 밖의 블록은 본 보고서 범위 밖이며, 학생이 작성하는 알고리즘 영역("Modify")에 해당합니다.

### 1.3 SID에 대한 주의사항
- Simulink SID는 **모델별 독립 ID 공간**이므로, 서로 다른 .slx 파일에서 같은 SID가 다른 블록을 가리키는 것은 정상입니다.
- 본 보고서의 "SID 충돌"은 **여러 Day 모델을 Final Project에 통합할 때** 발생할 잠재적 ID 충돌을 의미합니다.
- 충돌 발생 시 Final Project DNM 블록의 SID를 보존하고, Day 측 블록은 새 SID로 재할당해야 합니다.

## 2. 요약

| 항목 | 수량 |
|---|---:|
| Read CM Dict — Day1~6 | 28 |
| Read CM Dict — FP DNM | 5 |
| Read CM Dict — 합계 | 33 |
| Goto — Day1~6 | 107 |
| Goto — FP DNM | 8 |
| Goto — 합계 | 115 |
| **Read CM Dict SID 충돌** | **2개 SID** |
| **Goto SID 충돌** | **7개 SID (영향받는 Day 블록: 9개)** |

## 3. SID 충돌 요약 (Final Project DNM 우선)

### 3.1 Read CM Dict 충돌

| SID | ✅ 채택 (FP DNM) | ❌ 재할당 필요 (Day) |
|---:|---|---|
| **60** | `Final_Project/Read CM Dict3` — `Car.Fr1.rz` → `Ego_Global_Pos` | `Day6_Scenario_1/Read_Ego_V` — `Car.v` → `Ego_V` |
| **62** | `Final_Project/Read CM Dict5` — `Car.v` → `Ego_Velocity` | `Day6_Scenario_1/Read_Ego_X` — `Car.Fr1.tx` → `Ego_X` |

### 3.2 Goto 충돌

| SID | ✅ 채택 (FP DNM) | ❌ 재할당 필요 (Day) |
|---:|---|---|
| **30** | `Final_Project/Goto1` → `Parking_Start_Point_XY` | `Day3_Scenario_2/Goto7` → `Ego_Global_X`<br>`Day6_Scenario_1/Goto4` → `Target_Speed` |
| **31** | `Final_Project/Goto10` → `Ego_Velocity` | `Day3_Scenario_2/Goto8` → `Ego_Gloabl_Y`<br>`Day6_Scenario_1/Goto5` → `Traffic00_Global_Y` |
| **38** | `Final_Project/Goto4` → `Simulation_Time` | `Day3_Scenario_1/Goto12` → `Ego_Idx_Left` |
| **39** | `Final_Project/Goto5` → `Parking_Goal_Point` | `Day3_Scenario_1/Goto13` → `Ego_Valid_Left` |
| **40** | `Final_Project/Goto6` → `Parking_Map_Boundary` | `Day3_Scenario_1/Goto14` → `Local_Waypoints_Right` |
| **41** | `Final_Project/Goto7` → `Ego_Global_Pos` | `Day3_Scenario_1/Goto15` → `Ego_Idx_Right` |
| **42** | `Final_Project/Goto9` → `Obstacle_Info` | `Day3_Scenario_1/Goto16` → `Ego_Valid_Right` |

## 4. Final Project "Do Not Modify" 영역 상세

### 4.1 Read CM Dict (5개)

| SID | 블록 이름 | xname (CarMaker 신호) | 최종 도달 Goto Tag (BusCreator 경유 포함) |
|---:|---|---|---|
| 58 | `Read CM Dict1` | `Car.Fr1.tx` | `Ego_Global_Pos` |
| 59 | `Read CM Dict2` | `Car.Fr1.ty` | `Ego_Global_Pos` |
| 60 | `Read CM Dict3` | `Car.Fr1.rz` | `Ego_Global_Pos` |
| 61 | `Read CM Dict4` | `Time` | `Simulation_Time` |
| 62 | `Read CM Dict5` | `Car.v` | `Ego_Velocity` |

### 4.2 Goto (8개) — 입력 소스 포함

| SID | 블록 이름 | Goto Tag | 입력 소스 |
|---:|---|---|---|
| 30 | `Goto1` | `Parking_Start_Point_XY` | BusCreator(Const4=5.5, Const5=−36.5) |
| 31 | `Goto10` | `Ego_Velocity` | Read CM Dict5 (`Car.v`) |
| 38 | `Goto4` | `Simulation_Time` | Read CM Dict4 (`Time`) |
| 39 | `Goto5` | `Parking_Goal_Point` | BusCreator(Const6=35, Const7=−30, Const2=2π/3) |
| 40 | `Goto6` | `Parking_Map_Boundary` | Mux4 of 4 corners: (5.5,−4), (47.5,−4), (47.5,−45), (5.5,−45) → 42×41 m |
| 41 | `Goto7` | `Ego_Global_Pos` | BusCreator(Read CM Dict1, 2, 3) = (tx, ty, rz) |
| 42 | `Goto9` | `Obstacle_Info` | Subsystem (외부 `Traffic_Info_00`~`Traffic_Info_27` 28개를 BusSelector로 가공) |
| 3774 | `Goto` | `waypoints` | FromWorkspace (베이스 워크스페이스 `Waypoints` 변수) |

## 5. Read CM Dict 통합 표 (SID 순)

| SID | Origin | 모델 | 블록 이름 | xname | 연결 Goto Tag | 상태 |
|---:|---|---|---|---|---|---|
| 4 | Day | `Day1_Scenario_3_4` | `Read CM Dict` | `Car.Road.Path.DevDist` | `CrossTrackError` | unique |
| 8 | Day | `Day2_Scenario_1` | `Read CM Dict3` | `Car.v` | `Ego_Velocity` | unique |
| 14 | Day | `Day1_Scenario_1` | `Read CM Dict2` | `Car.v` | `Ego_Velocity` | unique |
| 14 | Day | `Day1_Scenario_2` | `Read CM Dict3` | `Car.v` | `Ego_Velocity` | unique |
| 25 | Day | `Day2_Scenario_4_5` | `Read CM Dict1` | `Car.Fr1.tx` | `Ego_Global_X` | unique |
| 26 | Day | `Day2_Scenario_4_5` | `Read CM Dict2` | `Car.Fr1.ty` | `Ego_Gloabl_Y` | unique |
| 27 | Day | `Day2_Scenario_4_5` | `Read CM Dict3` | `Car.Fr1.rz` | `Ego_Yaw` | unique |
| 28 | Day | `Day2_Scenario_2_3` | `Read CM Dict1` | `Car.Fr1.tx` | `Ego_Global_X` | unique |
| 28 | Day | `Day2_Scenario_4_5` | `Read CM Dict4` | `Car.v` | `Ego_Vx` | unique |
| 29 | Day | `Day2_Scenario_2_3` | `Read CM Dict2` | `Car.Fr1.ty` | `Ego_Gloabl_Y` | unique |
| 30 | Day | `Day2_Scenario_2_3` | `Read CM Dict3` | `Car.v` | `Ego_Velocity` | unique |
| 39 | Day | `Day3_Scenario_2` | `Read CM Dict1` | `Car.Fr1.tx` | `Ego_Global_X` | unique |
| 39 | Day | `Day6_Scenario_1` | `Read CM Dict` | `Car.Road.Path.DevDist` | `CrossTrackError` | unique |
| 40 | Day | `Day3_Scenario_2` | `Read CM Dict2` | `Car.Fr1.ty` | `Ego_Gloabl_Y` | unique |
| 40 | Day | `Day6_Scenario_1` | `Read CM Dict1` | `Traffic.T00.rzv` | `Traffic00_YawRate` | unique |
| 41 | Day | `Day3_Scenario_2` | `Read CM Dict3` | `Car.Fr1.rz` | `Ego_Yaw` | unique |
| 41 | Day | `Day6_Scenario_1` | `Read CM Dict2` | `Traffic.T01.rzv` | `Traffic01_YawRate` | unique |
| 42 | Day | `Day3_Scenario_2` | `Read CM Dict4` | `Car.v` | `Ego_Vx` | unique |
| 42 | Day | `Day6_Scenario_1` | `Read CM Dict3` | `Traffic.T02.rzv` | `Traffic02_YawRate` | unique |
| 58 | 🔵 FP_DNM | `Final_Project` | `Read CM Dict1` | `Car.Fr1.tx` | `Ego_Global_Pos` | unique |
| 59 | 🔵 FP_DNM | `Final_Project` | `Read CM Dict2` | `Car.Fr1.ty` | `Ego_Global_Pos` | unique |
| 60 | 🔵 FP_DNM | `Final_Project` | `Read CM Dict3` | `Car.Fr1.rz` | `Ego_Global_Pos` | ✅ **채택 (FP)** |
| 60 | Day | `Day6_Scenario_1` | `Read_Ego_V` | `Car.v` | `Ego_V` | ❌ **재할당 필요** |
| 61 | 🔵 FP_DNM | `Final_Project` | `Read CM Dict4` | `Time` | `Simulation_Time` | unique |
| 62 | 🔵 FP_DNM | `Final_Project` | `Read CM Dict5` | `Car.v` | `Ego_Velocity` | ✅ **채택 (FP)** |
| 62 | Day | `Day6_Scenario_1` | `Read_Ego_X` | `Car.Fr1.tx` | `Ego_X` | ❌ **재할당 필요** |
| 64 | Day | `Day6_Scenario_1` | `Read_Ego_Y` | `Car.Fr1.ty` | `Ego_Y` | unique |
| 66 | Day | `Day6_Scenario_1` | `Read_Ego_Yaw` | `Car.Fr1.rz` | `Ego_Yaw` | unique |
| 69 | Day | `Day3_Scenario_1` | `Read CM Dict1` | `Car.Fr1.tx` | `Ego_Global_X` | unique |
| 70 | Day | `Day3_Scenario_1` | `Read CM Dict2` | `Car.Fr1.ty` | `Ego_Gloabl_Y` | unique |
| 71 | Day | `Day3_Scenario_1` | `Read CM Dict3` | `Car.Fr1.rz` | `Ego_Yaw` | unique |
| 72 | Day | `Day3_Scenario_1` | `Read CM Dict4` | `Car.v` | `Ego_Vx` | unique |
| 3788 | Day | `Day4_5_Scenario_1` | `Read CM Dict1` | `Car.Fr1.tx,Car.Fr1.ty,Car.Fr1.rz,Car.vx` | `(none)` | unique |

## 6. Goto 통합 표

### 6.1 Final Project DNM Goto (8개, 모두 채택)

| SID | 블록 이름 | Tag |
|---:|---|---|
| **30** | `Goto1` | `Parking_Start_Point_XY` |
| **31** | `Goto10` | `Ego_Velocity` |
| **38** | `Goto4` | `Simulation_Time` |
| **39** | `Goto5` | `Parking_Goal_Point` |
| **40** | `Goto6` | `Parking_Map_Boundary` |
| **41** | `Goto7` | `Ego_Global_Pos` |
| **42** | `Goto9` | `Obstacle_Info` |
| **3774** | `Goto` | `waypoints` |

### 6.2 Day1~Day6 Goto — FP DNM과 충돌 없음 (98개, 그대로 유지 가능)

| SID | 모델 | 블록 (서브시스템 포함 상대경로) | Tag |
|---:|---|---|---|
| 2 | `Day1_Scenario_3_4` | `Goto1` | `CrossTrackError` |
| 5 | `Day2_Scenario_1` | `Goto3` | `Ego_Velocity` |
| 6 | `Day2_Scenario_1` | `Goto4` | `Target_Velocity` |
| 7 | `Day1_Scenario_1` | `Goto1` | `Ego_Velocity` |
| 7 | `Day1_Scenario_2` | `Goto1` | `Ego_Velocity` |
| 10 | `Day6_Scenario_1` | `Goto1` | `CrossTrackError` |
| 11 | `Day6_Scenario_1` | `Goto10` | `Traffic00_Yaw` |
| 12 | `Day2_Scenario_4_5` | `Goto1` | `Global_Waypoints` |
| 12 | `Day6_Scenario_1` | `Goto11` | `Traffic01_Yaw` |
| 13 | `Day2_Scenario_4_5` | `Goto2` | `Ego_Global_X` |
| 13 | `Day6_Scenario_1` | `Goto12` | `Traffic01_YawRate` |
| 14 | `Day2_Scenario_4_5` | `Goto3` | `Ego_Gloabl_Y` |
| 14 | `Day6_Scenario_1` | `Goto13` | `Traffic01_Global_X` |
| 15 | `Day2_Scenario_4_5` | `Goto4` | `Ego_Yaw` |
| 15 | `Day6_Scenario_1` | `Goto14` | `Traffic01_Global_Y` |
| 16 | `Day2_Scenario_4_5` | `Goto5` | `Local_Points` |
| 16 | `Day6_Scenario_1` | `Goto15` | `Traffic01_Vx` |
| 17 | `Day2_Scenario_4_5` | `Goto6` | `Coefficients` |
| 17 | `Day4_5_Scenario_1` | `Goto` | `Map` |
| 17 | `Day6_Scenario_1` | `Goto16` | `Traffic01_Vy` |
| 18 | `Day2_Scenario_4_5` | `Goto7` | `Ego_Vx` |
| 18 | `Day4_5_Scenario_1` | `Goto1` | `Finish_Point` |
| 18 | `Day6_Scenario_1` | `Goto17` | `Traffic01_Ax` |
| 19 | `Day2_Scenario_4_5` | `Goto8` | `LateralError` |
| 19 | `Day4_5_Scenario_1` | `Goto2` | `Traffic_size` |
| 19 | `Day6_Scenario_1` | `Goto18` | `Traffic01_Ay` |
| 20 | `Day2_Scenario_2_3` | `Goto1` | `Ego_Global_X` |
| 20 | `Day4_5_Scenario_1` | `Goto3` | `y` |
| 20 | `Day6_Scenario_1` | `Goto19` | `Traffic02_Yaw` |
| 21 | `Day2_Scenario_2_3` | `Goto2` | `Ego_Gloabl_Y` |
| 21 | `Day4_5_Scenario_1` | `Goto4` | `Start_Point` |
| 21 | `Day6_Scenario_1` | `Goto2` | `Traffic00_YawRate` |
| 22 | `Day2_Scenario_2_3` | `Goto3` | `Ego_Velocity` |
| 22 | `Day3_Scenario_2` | `Goto` | `Steering_Angle` |
| 22 | `Day4_5_Scenario_1` | `Goto6` | `Map_Boundary` |
| 22 | `Day6_Scenario_1` | `Goto20` | `Traffic02_YawRate` |
| 23 | `Day2_Scenario_2_3` | `Goto4` | `Traffic_Global_X` |
| 23 | `Day3_Scenario_2` | `Goto1` | `Local_Waypoints` |
| 23 | `Day4_5_Scenario_1` | `Goto9` | `Traffic_Info` |
| 23 | `Day6_Scenario_1` | `Goto21` | `Traffic02_Global_X` |
| 24 | `Day2_Scenario_2_3` | `Goto5` | `Traffic_Global_Y` |
| 24 | `Day3_Scenario_2` | `Goto10` | `Ego_Vx` |
| 24 | `Day6_Scenario_1` | `Goto22` | `Traffic02_Global_Y` |
| 25 | `Day2_Scenario_2_3` | `Goto7` | `Target_Distance` |
| 25 | `Day3_Scenario_2` | `Goto2` | `Distance` |
| 25 | `Day6_Scenario_1` | `Goto23` | `Traffic02_Vx` |
| 26 | `Day3_Scenario_2` | `Goto26` | `Traffic_Global_X` |
| 26 | `Day6_Scenario_1` | `Goto24` | `Traffic02_Vy` |
| 27 | `Day3_Scenario_2` | `Goto27` | `Traffic_Yaw` |
| 27 | `Day6_Scenario_1` | `Goto25` | `Traffic02_Ax` |
| 28 | `Day3_Scenario_2` | `Goto28` | `Traffic_Global_Y` |
| 28 | `Day6_Scenario_1` | `Goto26` | `Traffic02_Ay` |
| 29 | `Day3_Scenario_2` | `Goto29` | `Traffic_Vx` |
| 29 | `Day6_Scenario_1` | `Goto3` | `Traffic00_Global_X` |
| 32 | `Day3_Scenario_2` | `Goto9` | `Ego_Yaw` |
| 32 | `Day6_Scenario_1` | `Goto6` | `Traffic00_Vx` |
| 33 | `Day6_Scenario_1` | `Goto7` | `Traffic00_Vy` |
| 34 | `Day6_Scenario_1` | `Goto8` | `Traffic00_Ax` |
| 35 | `Day3_Scenario_1` | `Goto1` | `Line_Quality_Left` |
| 35 | `Day6_Scenario_1` | `Goto9` | `Traffic00_Ay` |
| 36 | `Day3_Scenario_1` | `Goto10` | `Ego_Vx` |
| 37 | `Day3_Scenario_1` | `Goto11` | `Local_Waypoints_Left` |
| 43 | `Day3_Scenario_1` | `Goto17` | `Local_Waypoints_X_Left` |
| 44 | `Day3_Scenario_1` | `Goto18` | `Coefficient_Left` |
| 45 | `Day3_Scenario_1` | `Goto19` | `Local_Waypoints_Y_Left` |
| 46 | `Day3_Scenario_1` | `Goto2` | `Waypoints_X_Left` |
| 47 | `Day3_Scenario_1` | `Goto20` | `Local_Waypoints_X_Right` |
| 48 | `Day3_Scenario_1` | `Goto21` | `Coefficient_Right` |
| 49 | `Day3_Scenario_1` | `Goto22` | `Local_Waypoints_Y_Right` |
| 50 | `Day3_Scenario_1` | `Goto23` | `Coefficient` |
| 51 | `Day3_Scenario_1` | `Goto24` | `State` |
| 52 | `Day3_Scenario_1` | `Goto25` | `Lateral_Error` |
| 53 | `Day3_Scenario_1` | `Goto3` | `Waypoints_Y_Left` |
| 54 | `Day3_Scenario_1` | `Goto4` | `Line_Quality_Right` |
| 55 | `Day3_Scenario_1` | `Goto5` | `Waypoints_X_Right` |
| 55 | `Day4_5_Scenario_1` | `Subsystem/Goto` | `Traffic_Info_01` |
| 56 | `Day3_Scenario_1` | `Goto6` | `Waypoints_Y_Right` |
| 56 | `Day4_5_Scenario_1` | `Subsystem/Goto1` | `Traffic_Info_02` |
| 57 | `Day3_Scenario_1` | `Goto7` | `Ego_Global_X` |
| 57 | `Day4_5_Scenario_1` | `Subsystem/Goto2` | `Traffic_Info_03` |
| 58 | `Day3_Scenario_1` | `Goto8` | `Ego_Gloabl_Y` |
| 58 | `Day4_5_Scenario_1` | `Subsystem/Goto3` | `Traffic_Info_04` |
| 59 | `Day3_Scenario_1` | `Goto9` | `Ego_Yaw` |
| 59 | `Day4_5_Scenario_1` | `Subsystem/Goto4` | `Traffic_Info_05` |
| 60 | `Day4_5_Scenario_1` | `Subsystem/Goto5` | `Traffic_Info_06` |
| 61 | `Day4_5_Scenario_1` | `Subsystem/Goto6` | `Traffic_Info_07` |
| 61 | `Day6_Scenario_1` | `Goto_Ego_V` | `Ego_V` |
| 63 | `Day6_Scenario_1` | `Goto_Ego_X` | `Ego_X` |
| 65 | `Day6_Scenario_1` | `Goto_Ego_Y` | `Ego_Y` |
| 67 | `Day6_Scenario_1` | `Goto_Ego_Yaw` | `Ego_Yaw` |
| 3777 | `Day6_Scenario_1` | `Goto27` | `Waypoints_X_Line1` |
| 3778 | `Day6_Scenario_1` | `Goto28` | `Waypoints_Y_Line1` |
| 3779 | `Day6_Scenario_1` | `Goto29` | `Waypoints_X_Line2` |
| 3780 | `Day6_Scenario_1` | `Goto30` | `Waypoints_Y_Line2` |
| 3783 | `Day6_Scenario_1` | `Goto31` | `Waypoints_X_Line3` |
| 3784 | `Day6_Scenario_1` | `Goto32` | `Waypoints_Y_Line3` |
| 3787 | `Day6_Scenario_1` | `Goto33` | `ego_lane_id` |
| 3788 | `Day6_Scenario_1` | `Goto34` | `ego_d` |

### 6.3 Day1~Day6 Goto — FP DNM과 SID 충돌 (재할당 필요)

| 충돌 SID | Day 모델 | Day 블록 | Day Tag (보존) | FP DNM이 같은 SID로 점유한 Tag |
|---:|---|---|---|---|
| **30** | `Day3_Scenario_2` | `Goto7` | `Ego_Global_X` | `Parking_Start_Point_XY` |
| **30** | `Day6_Scenario_1` | `Goto4` | `Target_Speed` | `Parking_Start_Point_XY` |
| **31** | `Day3_Scenario_2` | `Goto8` | `Ego_Gloabl_Y` | `Ego_Velocity` |
| **31** | `Day6_Scenario_1` | `Goto5` | `Traffic00_Global_Y` | `Ego_Velocity` |
| **38** | `Day3_Scenario_1` | `Goto12` | `Ego_Idx_Left` | `Simulation_Time` |
| **39** | `Day3_Scenario_1` | `Goto13` | `Ego_Valid_Left` | `Parking_Goal_Point` |
| **40** | `Day3_Scenario_1` | `Goto14` | `Local_Waypoints_Right` | `Parking_Map_Boundary` |
| **41** | `Day3_Scenario_1` | `Goto15` | `Ego_Idx_Right` | `Ego_Global_Pos` |
| **42** | `Day3_Scenario_1` | `Goto16` | `Ego_Valid_Right` | `Obstacle_Info` |

## 7. 통합 모델 마이그레이션 가이드

Day1~6의 기능을 Final Project 모델에 통합할 때 따라야 할 절차입니다.

### 7.1 점유된 SID (절대 사용 금지)

Final Project DNM이 이미 사용 중이므로 Day 측에서는 이 SID를 재사용할 수 없습니다.

| 카테고리 | 점유 SID |
|---|---|
| Read CM Dict | 58, 59, 60, 61, 62 |
| Goto | 30, 31, 38, 39, 40, 41, 42, 3774 |

### 7.2 재할당이 필요한 Day 블록

아래 블록들은 통합 시 새로운 SID로 재할당되어야 합니다.

**Read CM Dict (2건):**
- `Day6_Scenario_1/Read_Ego_V` (현재 SID=60) → 새 SID 필요
- `Day6_Scenario_1/Read_Ego_X` (현재 SID=62) → 새 SID 필요

**Goto (9건):**
- `Day3_Scenario_2/Goto7` (현재 SID=30, tag=`Ego_Global_X`)
- `Day3_Scenario_2/Goto8` (현재 SID=31, tag=`Ego_Gloabl_Y`)
- `Day6_Scenario_1/Goto4` (현재 SID=30, tag=`Target_Speed`)
- `Day6_Scenario_1/Goto5` (현재 SID=31, tag=`Traffic00_Global_Y`)
- `Day3_Scenario_1/Goto12` (현재 SID=38, tag=`Ego_Idx_Left`)
- `Day3_Scenario_1/Goto13` (현재 SID=39, tag=`Ego_Valid_Left`)
- `Day3_Scenario_1/Goto14` (현재 SID=40, tag=`Local_Waypoints_Right`)
- `Day3_Scenario_1/Goto15` (현재 SID=41, tag=`Ego_Idx_Right`)
- `Day3_Scenario_1/Goto16` (현재 SID=42, tag=`Ego_Valid_Right`)

> ℹ️ Simulink가 통합 시 자동으로 새 SID를 부여하므로, 수동 재할당은 일반적으로 불필요합니다. 다만 SID에 의존하는 외부 스크립트가 있다면 점검이 필요합니다.

### 7.3 신호 명명 정규화 권장 (별도 작업)

FP DNM이 사용하는 표준명에 맞춰 Day 측 신호명을 통일하는 것이 안전합니다.

| 의미 | Day 측 사용 명칭 (다양함) | FP DNM 표준 | 정규화 방향 |
|---|---|---|---|
| Ego 속도 | `Ego_Velocity`, `Ego_Vx`, `Ego_V` | `Ego_Velocity` | `Ego_Velocity`로 통일 |
| Ego 위치 (X, Y, Yaw 묶음) | 개별 발행 `Ego_Global_X` + `Ego_Gloabl_Y` + `Ego_Yaw` | `Ego_Global_Pos` (bus) | FP 표준 사용, 필요 시 BusSelector로 분해 |
| Cross-track error | `CrossTrackError` | (FP DNM 미발행, 알고리즘 직접 계산) | Day 표준 그대로 |
| 시뮬레이션 시간 | (미발행) | `Simulation_Time` | FP 표준 그대로 |

**오타 수정**: `Ego_Gloabl_Y` → `Ego_Global_Y` (Day2_Sc2_3, Day2_Sc4_5, Day3_Sc1, Day3_Sc2 — 4개 모델에서 동일 오타 발견)

## 8. 검증

본 보고서의 모든 SID, xname, Goto Tag 값은 MATLAB 런타임에서 `find_system` + `get_param`을 통해 **자동 수집**되었으며 사람의 수기 입력이 없습니다. 따라서 transcription 오류 가능성은 없습니다.

### 8.1 데이터 출처 명세

| 데이터 | 사용된 MATLAB API |
|---|---|
| Read CM Dict 식별 | `find_system(mn, 'BlockType', 'S-Function')` 후 `FunctionName == 'read_dict'` 필터 |
| Read CM Dict xname | `get_param(blk, 'xname')` |
| Goto SID/Tag | `find_system(mn, 'BlockType', 'Goto')` + `get_param(.., 'SID')` + `get_param(.., 'GotoTag')` |
| 연결 추적 | `PortHandles.Outport` → `get_param(line, 'DstBlockHandle')` |
| FP DNM 영역 판정 | annotation Position `[1975 1304 2624 2533]` + 블록 중심 좌표 비교 |

### 8.2 한계와 주의사항

- 본 보고서는 **활성 .slx 파일**만 분석했습니다. `*_backup_*.slx`는 제외했습니다.
- Final Project의 "Modify" 영역(annotation `Modify !!`, x: 2664~5572)은 분석 범위 밖입니다.
- Day 모델 내부의 일부 Goto는 **From이 존재하지 않는 orphan**입니다 (예: Day6의 28개 Traffic_* tag, Day3_Sc1의 5개 `Local_Waypoints_*`/`State` tag). 이는 별도 분석 보고서 참조.
- Day4_5의 Read CM Dict는 4채널 멀티 신호(`Car.Fr1.tx,ty,rz,vx`)를 한번에 읽어 Demux → MATLAB Function으로 직결되며 Goto를 거치지 않습니다.

## 9. 관련 보고서

- `day1_day6_read_cm_dict_goto_report.md` — Day1~Day6 Read CM Dict ↔ Goto 매핑 (정적 XML 파싱 기반)
- 본 보고서 — 위 보고서 + FP DNM 통합 + SID 충돌 분석 (런타임 기반)

---

*보고서 자동 생성: MATLAB 런타임 데이터 기반 (수기 입력 없음)*
