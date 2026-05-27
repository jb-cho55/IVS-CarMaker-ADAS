# Patch v2 — DEV_C_LaneChange_Tollgate.md

> 원본 §3 (LC 알고리즘)을 Werling Frenet Optimal Trajectory로 교체.
> 변경 근거: ADAS_Motion_Planning 프로젝트의 검증된 다중 후보 + 비용 최소 방식

---

## ▶ 변경 #1: S1: PLAN — 단일 quintic → Werling 다중 후보

### 기존 (변경 전)
```matlab
% 단일 quintic poly
d(s) = d0 + (d_target - d0) * (10s^3 - 15s^4 + 6s^5)
```

### 새 알고리즘 (Werling Frenet)

#### 1단계: Frenet 좌표계 변환
```matlab
% Frenet: 도로 따라가는 곡선 좌표계
% s = 도로 진행 방향 거리, d = 횡 offset
[s0, d0]     = global_to_frenet(EgoBus.x, EgoBus.y, target_lane_wps);
sd0          = EgoBus.vx;                       % 종속도 ≈ vx (도로 따라)
sdd0         = EgoBus.ax;
dd0          = EgoBus.vy;                       % 횡속도
ddd0         = 0;
```

#### 2단계: 다중 후보 생성 (Nd × Nt)
```matlab
target_d = LC_RequestBus.target_d;  % 목표 lane 중심의 d
target_v = MissionBus.target_speed;

trajectories = {};  % cell array of candidates
for df = DF_SET                     % 횡 종착점 후보 (target_d ± offset)
    d_f = target_d + df;
    for T = MIN_T:DT_T:MAX_T        % 시간 후보
        % ─── 횡방향: Quintic Polynomial (final position 고정) ──
        % 양끝 조건: [d0, dd0, ddd0]  vs  [d_f, 0, 0]
        lat_poly = quintic_polynomial(d0, dd0, ddd0, d_f, 0, 0, T);

        % ─── 종방향: Quartic Polynomial (final position 자유) ──
        % 양끝 조건: [s0, sd0, sdd0]  vs  [-, target_v, 0]  ← s_f 자유
        long_poly = quartic_polynomial(s0, sd0, sdd0, target_v, 0, T);

        % trajectory sampling (TS_CONTROL 간격)
        t_arr  = 0 : TS_CONTROL : T;
        d_arr  = polyval_d(lat_poly, t_arr);
        dd_arr = polyval_dd(lat_poly, t_arr);
        ddd_arr= polyval_ddd(lat_poly, t_arr);
        s_arr  = polyval_s(long_poly, t_arr);
        sd_arr = polyval_sd(long_poly, t_arr);
        sddd_arr = polyval_sddd(long_poly, t_arr);

        % ─── 비용 계산 ──
        c_lat = K_J*sum(ddd_arr.^2)*TS_CONTROL + K_T*T + K_D*(d_f - target_d)^2;
        c_lon = K_J*sum(sddd_arr.^2)*TS_CONTROL + K_T*T + K_V*(sd_arr(end) - target_v)^2;
        c_ttc = K_TTC / max(LaneRiskBus.ttc(target_lane), 0.5);
        c_tot = K_LAT*c_lat + K_LON*c_lon + c_ttc;

        % ─── 제약 검증 ──
        if max(abs(sd_arr)) > V_MAX_FRENET, continue; end
        a_total = sqrt(sd_arr.*sd_arr + dd_arr.*dd_arr);  % 가속도 크기
        if max(a_total) > ACC_MAX_FRENET, continue; end
        kappa = compute_curvature(s_arr, d_arr);          % yaw_diff/ds
        if max(abs(kappa)) > K_MAX_CURV, continue; end

        trajectories{end+1} = struct('cost', c_tot, ...
                                     's', s_arr, 'd', d_arr, ...
                                     'T', T, 'd_f', d_f);
    end
end
```

#### 3단계: 최소 비용 trajectory 선택
```matlab
if isempty(trajectories)
    % 모든 후보가 제약 위반 → ABORT
    state = S5_ABORT;
    LC_StatusBus.error_code = ERR_NO_VALID_TRAJ;
    return;
end

costs = cellfun(@(t) t.cost, trajectories);
[~, idx_min] = min(costs);
best = trajectories{idx_min};

% Frenet → Global 변환 후 LC_TrajectoryBus 채움
[x_arr, y_arr, yaw_arr] = frenet_to_global(best.s, best.d, target_lane_wps);
LC_TrajectoryBus.valid(1:length(x_arr)) = true;
LC_TrajectoryBus.x(1:length(x_arr))     = x_arr;
LC_TrajectoryBus.y(1:length(y_arr))     = y_arr;
LC_TrajectoryBus.yaw(1:length(yaw_arr)) = yaw_arr;
```

---

## ▶ 변경 #2: Quintic / Quartic Polynomial 헬퍼

### 횡방향 — Quintic (5차, 양끝 위치 모두 고정)
```matlab
function poly = quintic_polynomial(x0, v0, a0, xf, vf, af, T)
  % 6 boundary conditions → 6 coefficients (a0..a5)
  A = [T^3,    T^4,    T^5;
       3*T^2,  4*T^3,  5*T^4;
       6*T,    12*T^2, 20*T^3];
  b = [xf - x0 - v0*T - 0.5*a0*T^2;
       vf - v0 - a0*T;
       af - a0];
  a345 = A \\ b;
  poly = [x0, v0, 0.5*a0, a345(1), a345(2), a345(3)];
end
```

### 종방향 — Quartic (4차, 종착 위치 자유, 속도/가속도만 제약)
```matlab
function poly = quartic_polynomial(s0, v0, a0, vf, af, T)
  % 5 boundary conditions → 5 coefficients (a0..a4)
  A = [3*T^2, 4*T^3;
       6*T,   12*T^2];
  b = [vf - v0 - a0*T;
       af - a0];
  a34 = A \\ b;
  poly = [s0, v0, 0.5*a0, a34(1), a34(2)];
end
```

**왜 종방향이 Quartic인가**: 차선 변경 시 어디까지 갈지(s_f)는 자유, 목표 속도(v_f)와 가속도(a_f=0)만 정해두는 게 자연스러움. 위치 제약을 없애면 더 부드러운 가속 프로파일 생성.

---

## ▶ 변경 #3: 곡률 제약 검증 (안전성)

```matlab
function kappa = compute_curvature(s, d)
  % yaw_i = atan2(diff(d), diff(s)) (Frenet frame yaw)
  ds = diff(s);
  yaw_arr = atan2(diff(d), ds);
  % yaw wrap-around 보정
  dyaw = diff(yaw_arr);
  dyaw(dyaw >  pi) = dyaw(dyaw >  pi) - 2*pi;
  dyaw(dyaw < -pi) = dyaw(dyaw < -pi) + 2*pi;
  kappa = [0; dyaw./ds(1:end-1); 0];
end
```

트리거: `max(abs(kappa)) > K_MAX_CURV` → 해당 후보 reject

---

## ▶ 07_ParkingPlanner에도 동일 적용

`DEV_D` 주차 trajectory에도 같은 패턴 권장:
- 횡 Quintic / 종 Quartic 분리
- 곡률 제약 검증 `|κ| ≤ K_MAX_CURV` (주차장에선 K_MAX 더 크게, 예: 0.5)
- 후진 trajectory도 마찬가지 (단, sd0이 음수 가능)

---

## ▶ 실시간성 분석

### 계산량
- Nd = 3 (DF_SET), Nt = 6 (MIN_T 2.0 ~ MAX_T 4.5 / DT_T 0.5)
- 후보 수 = 18개
- 각 후보: T=4.5s × 100Hz = 450 sampling 점, polynomial 평가 + 비용 계산
- 예상 계산 시간: ~5ms (R2024a, i7급)
- TS_CONTROL = 10ms이므로 여유 충분

### 호출 빈도
- PLAN 단계는 S1 진입 시 1회만 (또는 LC_REPLAN_PERIOD = 0.5s 마다 재계획)
- EXECUTE 단계는 미리 계산된 trajectory만 추종 (계산량 거의 없음)
