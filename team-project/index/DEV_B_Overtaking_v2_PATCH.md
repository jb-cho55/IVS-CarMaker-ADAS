# Patch v2 — DEV_B_Overtaking.md

> 원본 §2 (State Machine S1) 알고리즘과 §3 (DECIDE_TARGET_LANE 로직)을 다음으로 교체.
> 변경 근거: `safe_left/safe_right` 이분법 → `lane_cost` argmin (Werling 영감)

---

## ▶ 변경: S1: DECIDE_TARGET_LANE — lane_cost 기반

### 기존 (변경 전)
```matlab
if safe_left
    target_lane = current_lane - 1;
elseif safe_right
    target_lane = current_lane + 1;
end
```

### 새 로직 (변경 후)
```matlab
%% 1) 후보 차선 = 현재 ± 1 (인접 차선만)
cand = [current_lane - 1, current_lane + 1];
cand = cand(cand >= 1 & cand <= MAX_LANE);

%% 2) 후보별 안전성 게이트 (반드시 만족해야 후보 자격)
valid = false(size(cand));
for k = 1:length(cand)
    L = cand(k);
    valid(k) = (LaneRiskBus.front_gap(L) > front_gap_min) && ...
               (LaneRiskBus.rear_gap(L)  > rear_gap_min)  && ...
               (LaneRiskBus.ttc(L) > TTC_min) && ...
               (~LaneRiskBus.lane_blocked(L));
end
valid_cand = cand(valid);

%% 3) 안전한 후보 중 lane_cost 최소 차선 선택
if isempty(valid_cand)
    target_lane = current_lane;  % 그대로 유지
    state = S0_IDLE;
else
    costs = LaneRiskBus.lane_cost(valid_cand);
    [~, idx_min] = min(costs);
    target_lane = valid_cand(idx_min);
    state = S2_REQUEST_LANE_CHANGE;
end
```

### 효과

| 항목 | 변경 전 | 변경 후 |
|---|---|---|
| 좌/우 둘 다 안전한 경우 | 좌 우선 (임의) | TTC/gap 최적 차선 |
| 2차선 이상 변경 필요 시 | 불가 | 현재는 인접 차선만, 향후 확장 가능 |
| MissionSupervisor와의 결합 | 별도 | `target_lane`만 전달, MS가 LC_RequestBus 생성 |

---

## ▶ 입력 인터페이스 확인

기존 §1 입력 표 그대로 (변경 없음). 다만 `LaneRiskBus.lane_cost` 사용을 명시:

| Bus | 사용 field | 용도 |
|---|---|---|
| **LaneRiskBus** | ttc[4], front_gap[4], rear_gap[4], **lane_cost[4]**, lane_blocked[4], safe_left, safe_right | 위험도 + 비용 |

---

## ▶ 의사결정 vs 안전성 게이트

**원칙**: `lane_cost`는 최적화 score, 안전성 게이트(`front_gap > min` 등)는 hard constraint.
비용이 낮더라도 안전 임계값을 위반하면 후보에서 제외.

```
hard constraint (안전성) → 통과한 후보만 → soft optimization (lane_cost min)
```

이렇게 분리하면 `lane_cost` 가중치 튜닝 실수로 위험한 차선을 선택하는 것을 방지.
