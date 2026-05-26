function [target_lane, lane_cost] = dev_d_parking(ctrl_bus)
%#codegen
% PATH_PLANNING_FCN — 차선 추천 + 주차 경로
% Owner: Dev-D (시나리오 2-1 주차)
%
% Outputs:
%   target_lane : int32, 추천 차선 (1~4)
%   lane_cost   : double [4x1], 각 차선 비용

% ===== ctrl_bus 추출 =====
ego_x        = ctrl_bus.ego_x;
long_dist    = ctrl_bus.long_dist;
lat_dist     = ctrl_bus.lat_dist;
current_lane = ctrl_bus.current_lane;
LANE_WIDTH   = ctrl_bus.lane_width;

% ===== 상수 =====
MAX_LANE  = int32(4);
K_LATBIAS = 5.0;
K_BLOCK   = 1000.0;
K_FRONT   = 100.0;
BLOCK_DIST_LONG = 10.0;
BLOCK_DIST_LAT  = 1.5;
FRONT_DIST_MAX  = 50.0;
TG_START_X      = 500.0;
TG_END_X        = 580.0;
TG_GATE_LANE    = int32(2);

lane_cost = zeros(4, 1);
cl = double(current_lane);

% ===== Step 1: 기본 비용 (current 차선 근접 선호) =====
for i = 1:4
    lane_cost(i) = abs(double(i) - cl) * K_LATBIAS;
end

% ===== Step 2: 인접 차선만 후보 (current ± 1) =====
for i = 1:4
    if abs(double(i) - cl) > 1
        lane_cost(i) = lane_cost(i) + 9999;
    end
end

% ===== Step 3: 앞차 패널티 (현재 차선) =====
if long_dist > 0.5 && long_dist < FRONT_DIST_MAX && abs(lat_dist) < 1.8
    lane_cost(current_lane) = lane_cost(current_lane) + K_FRONT;
end

% ===== Step 4: 옆 차선 안전 (트래픽 근접 → block) =====
for i = 1:4
    expected_lat = (double(i) - cl) * LANE_WIDTH;
    if abs(lat_dist - expected_lat) < BLOCK_DIST_LAT && abs(long_dist) < BLOCK_DIST_LONG
        lane_cost(i) = lane_cost(i) + K_BLOCK;
    end
end

% ===== Step 5: 톨게이트 시나리오 — 게이트 차선 강제 =====
if ego_x > TG_START_X && ego_x < TG_END_X
    lane_cost(TG_GATE_LANE) = 0;
    for i = 1:4
        if int32(i) ~= TG_GATE_LANE
            lane_cost(i) = lane_cost(i) + 500;
        end
    end
end

% ===== 최저 비용 차선 =====
[~, idx] = min(lane_cost);
target_lane = int32(idx);
if target_lane > MAX_LANE, target_lane = MAX_LANE; end
if target_lane < int32(1), target_lane = int32(1); end
end
