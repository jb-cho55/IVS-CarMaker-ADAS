# 📋 DEV Tasks v2 — 각 팀원 작업 명세

> 자기 섹션만 읽고 작업 시작. 다른 dev 섹션은 인터페이스 확인용.
> 공통: `setup_paths.m` 실행 후 자기 `.m` 수정 → CarMaker 시뮬 테스트.

---

# 🚗 Dev-A — Longitudinal Controller

**파일**: `functions/longitudinal_controller_fcn.m`
**Lib**: `Lib_LongitudinalController` (10ms)

### 시그니처 (절대 변경 금지)
```matlab
function acc_cmd = longitudinal_controller_fcn(v_ego, v_target, front_gap, mode)
```

### 현재 입력 source (Final_Project.slx에서)
| 입력 | 현재 source | TODO (다른 Lib와 연결되면 자동) |
|---|---|---|
| `v_ego` | From[Ego_Velocity] (실제값) | OK |
| `v_target` | Constant 20 m/s | 추후 MissionSupervisor 출력에 연결 |
| `front_gap` | Constant 999 m | 추후 Dev-D의 PathPlanning이 제공 |
| `mode` | From[Mission_Mode] (Dev-B 출력) | OK |

### 구현 권장
1. **속도 PID**: e_v = v_target - v_ego, PID로 acc_cmd 산출
2. **ACC**: front_gap < safe_dist 일 때 감속 (e_d = front_gap - safe_dist)
3. **곡률 감속**: mode에 따라 곡선부 진입 시 v_target 감소
4. **비상 제동**: front_gap < 5m 일 때 acc = -5
5. **출력 한계**: [-5, +3] m/s² (Saturation도 외부에서 한 번 더 적용됨)

### 단위 테스트
```matlab
acc1 = longitudinal_controller_fcn(10, 20, 50, int32(1));  %% 정상 가속
assert(acc1 > 0 && acc1 < 3);
acc2 = longitudinal_controller_fcn(20, 20, 3, int32(1));   %% 비상 제동
assert(acc2 == -5);
```

---

# 🏎 Dev-B — Mode Manager

**파일**: `functions/mode_manager_fcn.m`
**Lib**: `Lib_ModeManager` (10ms)

### 시그니처
```matlab
function mode = mode_manager_fcn(ego_v, ego_x, ego_y, ego_yaw, ...
                                  traffic_x, traffic_y, target_lane, current_lane)
```

### 현재 입력 source
| 입력 | 현재 source | TODO |
|---|---|---|
| `ego_v` | From[Ego_Velocity] | OK |
| `ego_x/y/yaw` | From[Ego_Global_X/Y/Yaw] | OK |
| `traffic_x` | Constant 999 (멀리) | Obstacle_Info Bus 처리 시 실제값 |
| `traffic_y` | Constant 0 | 위 동일 |
| `target_lane` | From[Target_Lane] (Dev-D 출력) | OK |
| `current_lane` | Constant int32(1) | 자차 lane assign 모듈 필요 |

### 구현 권장
1. **State Machine** (persistent state 변수):
   - LK → LC_OUT (front 차량 가까움 + safe_left/right)
   - LC_OUT → LC_PASS (4초 경과)
   - LC_PASS → LC_BACK (추월 완료)
   - LC_BACK → LK (4초 경과)
2. **모드 우선순위**: Emergency > Parking > Tollgate > LC > LK
3. **mode를 int32로 반환** (enum value)

### v2 patch 참고: `DEV_B_Overtaking_v2_PATCH.md`
- lane_cost argmin으로 target_lane 선택 (Dev-D 협업)
- 안전성 hard constraint + 비용 soft optimization 분리

---

# 🔄 Dev-C — Trajectory Planning

**파일**: `functions/trajectory_planning_fcn.m`
**Lib**: `Lib_TrajectoryPlanning` (100ms)

### 시그니처
```matlab
function cte_offset = trajectory_planning_fcn(cte_in, mode, ego_x, ego_y, ego_yaw, ego_v)
```

### 현재 입력 source
| 입력 | source |
|---|---|
| `cte_in` | Constant 0 (DevDist Read 추가 시 실제값) |
| `mode` | From[Mission_Mode] |
| `ego_x/y/yaw/v` | From[Ego_Global_X/Y/Yaw/Velocity] |

### 출력
- `cte_offset` → Goto[Trajectory_CTE] → Dev-E의 lateral_controller 입력

### 구현 권장 (v2 patch 적용)
1. **현재**: 단순 quintic offset ramp (LC 모드일 때 차선 변경 offset)
2. **v2**: **Werling Frenet** 다중 후보 + 비용 최소 선택
   - 횡: Quintic poly (양끝 위치 고정)
   - 종: Quartic poly (final position 자유)
   - Nd × Nt 후보 → c_tot 최소 선택
   - 곡률 제약 검증: |κ| > K_MAX → reject
3. 상세: `DEV_C_LaneChange_Tollgate_v2_PATCH.md`

### 100ms 주기 — 실시간 가능성
- Werling 18개 후보 계산: ~1.2 ms (CPU 12% 사용)
- 여유 충분

---

# 🅿 Dev-D — Path Planning

**파일**: `functions/path_planning_fcn.m`
**Lib**: `Lib_PathPlanning` (100ms)

### 시그니처
```matlab
function [target_lane, lane_cost] = path_planning_fcn(ego_x, ego_y, ego_yaw, ...
                                                       traffic_x, traffic_y, current_lane)
```

### 출력 명세
- `target_lane` : int32, 1~4 (인접 차선만 권장: current ± 1)
- `lane_cost`   : double [4×1], 각 차선의 비용 (낮을수록 좋음)

### 구현 권장 (v2 patch 적용)
```matlab
lane_cost(i) = K_BLOCK * blocked(i) ...
             + K_TTC   / max(ttc(i), 0.5) ...
             + K_GAP   / max(front_gap(i), 5.0) ...
             + K_LATBIAS * abs(i - current_lane);

%% argmin으로 target_lane
[~, target_lane] = min(lane_cost);
```

### TODO (협업)
- `Obstacle_Info` Bus 처리 (Dev-F가 BusSelector 추가 예정)
- `current_lane` 계산 모듈 (자차 글로벌 위치 → lane 인덱스)
- 상세: `04_System_Interface_v2_PATCH.md` LaneRiskBus 정의

---

# 🎯 Dev-E — Lateral Controller

**파일**: `functions/lateral_controller_fcn.m`
**Lib**: `Lib_LateralController` (10ms)

### 시그니처
```matlab
function steer_cmd = lateral_controller_fcn(cte, mode)
```

### 현재 입력
| 입력 | source |
|---|---|
| `cte` | From[Trajectory_CTE] (Dev-C 출력) |
| `mode` | From[Mission_Mode] (Dev-B 출력) |

### 구현 권장
1. **PID** (현재): Kp=1.5, Ki=0.3, Kd=0.4, anti-windup ±3.0
2. **Pure Pursuit 추가**: cte → lookahead point → steering geometry
3. **Stanley 컨트롤러**: yaw error + cte 결합
4. **mode별 게인 조정**: LK는 부드럽게, LC는 빠르게, Parking은 정밀
5. **출력 한계**: ±π/6 rad (Saturation도 외부 한 번 더)

### Day6에서 검증된 PID 게인 (출발점)
- Kp=1.5, Ki=0.3, Kd=0.4
- 곡선에서 path yaw feedforward 시도 가능 (단, Path.DevAng 신호 추가 필요 → Dev-F)

---

## 공통: 외부 함수 작성 규칙

1. **`%#codegen` directive 필수**
   ```matlab
   function out = my_fcn(in1, in2)
   %#codegen
   % ... code ...
   end
   ```

2. **persistent 변수 초기화 패턴**
   ```matlab
   persistent state e_prev
   if isempty(state), state = int32(0); e_prev = 0; end
   ```

3. **고정 크기 배열만**
   ```matlab
   % OK: lane_cost = zeros(4, 1);
   % NO: lane_cost = zeros(MAX_LANE, 1);  % variable
   ```

4. **타입 명시**
   ```matlab
   state = int32(1);   % int32
   acc_cmd = 0.5;      % double (default)
   ```

5. **금지 함수**
   - `arrayfun`, `cellfun` (일부 codegen 미지원)
   - `eval`, `feval` 동적 호출
   - 일부 stats/optim toolbox 함수
