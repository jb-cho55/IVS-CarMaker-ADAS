function ctrl_bus = dev_f_supervisor(ego_v, ego_x, ego_y, ego_yaw, ...
                                    cte_raw, traffic_x, traffic_y, traffic_v, ...
                                    v_target_user)
%#codegen
% SUPERVISOR_FCN — 모든 raw 신호 정리 → Bus(struct) 생성
% Owner: Dev-F (Supervisor / 인프라)
%
% 역할:
%   1) CarMaker raw 신호 → 정리된 struct (ctrl_bus)로 묶음
%   2) Frenet 좌표 변환 (트래픽 → 자차 기준 종/횡 거리)
%   3) Placeholder 처리 (트래픽 없음 → 999)
%   4) current_lane 추정 (자차 글로벌 위치 → lane 인덱스)
%   5) front_gap 계산 (같은 차선 트래픽만)
%
% Inputs (CarMaker Read Dict + 사용자 설정):
%   ego_v, ego_x, ego_y, ego_yaw : 자차 상태
%   cte_raw                       : Road.DevDist (차로 중앙 오차)
%   traffic_x, traffic_y          : 가장 위협적인 트래픽 위치
%   traffic_v                     : 트래픽 속도
%   v_target_user                 : 사용자 설정 목표 속도 (기본 20)
%
% Output:
%   ctrl_bus : struct (Bus 호환). 모든 functions가 이 struct를 입력으로 받음.

% ===== 상수 =====
LANE_WIDTH = 3.0;
MAX_LANE = 4;
LANE_CENTER_Y = [-4.5, -1.5, 1.5, 4.5];  % 각 차선 중앙의 글로벌 Y (예시)

% ===== 1) Frenet 좌표 변환 (트래픽 → 자차 기준) =====
dx = traffic_x - ego_x;
dy = traffic_y - ego_y;
long_dist = cos(ego_yaw)*dx + sin(ego_yaw)*dy;
lat_dist  = -sin(ego_yaw)*dx + cos(ego_yaw)*dy;

% ===== 2) Placeholder 처리 (트래픽 없음 = (0,0) 가정) =====
if traffic_x == 0 && traffic_y == 0
    long_dist = 999.0;
    lat_dist  = 0.0;
end

% ===== 3) front_gap 계산 (같은 차선 + 앞쪽 트래픽만) =====
if long_dist > 0.5 && abs(lat_dist) < 1.8
    front_gap = long_dist;
else
    front_gap = 999.0;  % 같은 차선에 앞차 없음
end

% ===== 4) current_lane 추정 (글로벌 Y → lane 인덱스) =====
lane_dists = abs(LANE_CENTER_Y - ego_y);
[~, lane_idx] = min(lane_dists);
current_lane = int32(lane_idx);
if current_lane > int32(MAX_LANE), current_lane = int32(MAX_LANE); end
if current_lane < int32(1),         current_lane = int32(1);        end

% ===== 5) ctrl_bus 구조체 생성 (Bus Object 호환) =====
ctrl_bus = struct( ...
    'ego_v',         double(ego_v), ...
    'ego_x',         double(ego_x), ...
    'ego_y',         double(ego_y), ...
    'ego_yaw',       double(ego_yaw), ...
    'cte_in',        double(cte_raw), ...
    'traffic_x',     double(traffic_x), ...
    'traffic_y',     double(traffic_y), ...
    'traffic_v',     double(traffic_v), ...
    'long_dist',     double(long_dist), ...
    'lat_dist',      double(lat_dist), ...
    'front_gap',     double(front_gap), ...
    'current_lane',  current_lane, ...
    'v_target',      double(v_target_user), ...
    'lane_width',    double(LANE_WIDTH));
end
