# functions/ — 외부 알고리즘 함수 (Git 충돌 회피 핵심)

## 핵심 원칙

- 각 `.m` 파일은 **단일 담당자**가 소유
- Final_Project `.slx`는 **거의 수정 안 함** (한 번 셋업 후 봉인)
- 알고리즘 변경은 **이 폴더의 `.m` 파일만** 수정
- `.m` 파일은 텍스트라서 git diff/merge 가능 → 충돌 회피

## 파일 소유권

| 파일 | 담당 Dev | 04 아키텍처 매핑 | Lib 위치 |
|---|:-:|---|---|
| `mode_manager_fcn.m` | **Dev-B** | 02_MissionSupervisor + 04_OvertakingDecision | `Lib_ModeManager` |
| `path_planning_fcn.m` | **Dev-D** | 01_WorldModel + 07_ParkingPlanner | `Lib_PathPlanning` |
| `trajectory_planning_fcn.m` | **Dev-C** | 05_LaneChangeController + 06_TollgateHighPass | `Lib_TrajectoryPlanning` |
| `longitudinal_controller_fcn.m` | **Dev-A** | 03_LaneKeeping_ACC (종방향) + 08_ParkingController (종) | `Lib_LongitudinalController` |
| `lateral_controller_fcn.m` | **Dev-E** | 03_LaneKeeping_ACC (횡방향) + 08_ParkingController (횡) | `Lib_LateralController` |

Dev-F (Supervisor): 통합, 검증, 메인 `.slx` 관리. 알고리즘 함수는 직접 수정 안 함.

## 인터페이스 (시그니처 변경 시 합의 필요)

### `lateral_controller_fcn(cte, mode) → steer_cmd`
- `cte` : Cross-Track Error [m]
- `mode` : int32, mission mode
- `steer_cmd` : 앞바퀴 조향각 [rad], 범위 ±π/6

### `longitudinal_controller_fcn(v_ego, v_target, front_gap, mode) → acc_cmd`
- `v_ego` : 자차 속도 [m/s]
- `v_target` : 목표 속도 [m/s]
- `front_gap` : 선행차 거리 [m] (없으면 999)
- `acc_cmd` : 목표 가속도 [m/s²], 범위 [-5, +3]

### `trajectory_planning_fcn(cte_in, mode, ego_x, ego_y, ego_yaw, ego_v) → cte_offset`
- `cte_in` : 원본 DevDist
- `mode` : LK/LC_OUT/LC_PASS/LC_BACK 등
- `cte_offset` : 보정된 CTE (lateral controller 입력)

### `path_planning_fcn(ego_x, ego_y, ego_yaw, traffic_x, traffic_y, current_lane) → [target_lane, lane_cost]`
- `lane_cost` : double [4×1], 차선별 비용 (낮을수록 좋음)
- `target_lane` : int32, 목표 차선 (argmin)

### `mode_manager_fcn(ego_v, ego_x, ego_y, ego_yaw, traffic_x, traffic_y, target_lane, current_lane) → mode`
- `mode` : int32, 1=LK, 2=OT, 3=LC_OUT, 4=LC_PASS, 5=LC_BACK, 6=TG, 7=PK, 9=EMG

## 코딩 규칙

1. **`%#codegen`** directive 필수 (MATLAB Function block에서 호출 가능하도록)
2. **고정 크기 배열**만 사용 (variable-size 금지)
3. **persistent 변수** OK, 초기화는 `if isempty(...)` 패턴
4. **MATLAB toolbox 함수** 중 codegen 지원 안 되는 것 금지 (예: `arrayfun`, `cellfun`, 일부 stats)
5. **시그니처 변경 시** Slack/PR review 필수 (다른 Dev의 Lib와 연동되므로)

## 새 작업자 셋업

```matlab
cd D:\\HL_IVS_School\\...\\src_cm4sl
cmenv               %% CarMaker 환경
addpath('functions') %% 이 폴더 추가
open generic_IVS.mdl
```

또는 `setup_paths.m` 실행 (cmenv + functions path 자동).
