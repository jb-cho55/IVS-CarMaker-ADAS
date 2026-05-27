# Patch v2 — 04_Final_Project_System_Interface_Architecture.md

> 원본 문서의 아래 섹션을 다음 내용으로 교체하세요.
> 변경 근거: ADAS_Motion_Planning 프로젝트의 Werling Frenet 기법 + lane_cost 함수 도입

---

## ▶ 변경 #1: §5.5 LaneRiskBus — lane_cost 명시 (원본 교체)

| Field | Type/Dim | Unit / Meaning |
|---|---:|---|
| `front_gap` | double `[MAX_LANE,1]` | m |
| `rear_gap` | double `[MAX_LANE,1]` | m |
| `front_rel_v` | double `[MAX_LANE,1]` | m/s |
| `rear_rel_v` | double `[MAX_LANE,1]` | m/s |
| `side_gap` | double `[MAX_LANE,1]` | m |
| `ttc` | double `[MAX_LANE,1]` | s |
| `lane_blocked` | boolean `[MAX_LANE,1]` |  |
| **`lane_cost`** | double `[MAX_LANE,1]` | **Σ(K_TTC/ttc + K_GAP/front_gap + K_BLOCK·blocked + K_LATBIAS·\\|lane-current\\|)** |
| `cut_in_risk` | double `[MAX_LANE,1]` |  |
| `boundary_risk` | double `[MAX_LANE,1]` |  |
| `safe_left` | boolean scalar |  |
| `safe_right` | boolean scalar |  |

### lane_cost 계산식 (01_WorldModel 책임)
```matlab
for i = 1:MAX_LANE
    lane_cost(i) = K_BLOCK * double(lane_blocked(i)) ...
                 + K_TTC   / max(ttc(i), 0.5) ...
                 + K_GAP   / max(front_gap(i), 5.0) ...
                 + K_LATBIAS * abs(i - double(EgoBus.lane_idx));
end
```

**용도**: 04_Overtaking, 02_MissionSupervisor에서 `argmin(lane_cost)`로 최적 차선 자동 선택.

---

## ▶ 변경 #2: §10 Constants — Werling/비용 상수 추가

기존 상수 블록 끝에 다음을 추가하세요:

```matlab
%% ====================================================================
%% Lane Cost weights (01_WorldModel → LaneRiskBus.lane_cost)
%% ====================================================================
K_BLOCK    = 1000.0;   % blocked 차선은 사실상 금지
K_TTC      = 50.0;     % TTC 짧을수록 비용 ↑
K_GAP      = 30.0;     % gap 좁을수록 비용 ↑
K_LATBIAS  = 5.0;      % 멀리 떨어진 차선 변경 disinencentive

%% ====================================================================
%% Werling Frenet (05_LaneChange, 07_ParkingPlanner)
%% ====================================================================
%% 횡방향 (Quintic) 후보
DF_SET     = [-1.5, 0, 1.5];   % 현재 d 기준 횡 목표점 후보 [m]
MIN_T      = 2.0;              % 최소 trajectory 시간 [s]
MAX_T      = 4.5;              % 최대 trajectory 시간 [s]
DT_T       = 0.5;              % 시간 후보 간격 [s]

%% 비용 가중치
K_J        = 0.1;     % jerk 적분 (부드러움)
K_T        = 1.0;     % 짧은 시간 선호
K_D        = 1.0;     % target_d 추종 (횡)
K_V        = 1.0;     % target_v 추종 (종)
K_LAT      = 1.0;     % 횡 vs 종 가중
K_LON      = 1.0;

%% Werling 제약
V_MAX_FRENET = 30.0;  % m/s (108 km/h)
ACC_MAX_FRENET = 4.0; % m/s²
K_MAX_CURV = 0.20;    % 1/m (turn radius 5m)
```

---

## ▶ 변경 #3: §11 Required Goto Tags — 정규화 노트 추가

기존 표 아래 다음 경고를 추가:

> ⚠️ **Day1~3 모델에 `Ego_Gloabl_Y` 오타가 존재합니다** (Day2/3 시나리오 모두).
> 00_InputAdapter에서 반드시 `Ego_Global_Y`로 **정규화**해야 합니다.
> Feature 모듈은 표준 이름 `Ego_Global_Y`만 사용 (오타 이름 사용 금지).
