function fp_define_buses()
%FP_DEFINE_BUSES Final Project Bus 객체 base workspace 등록
MAX_LANE=4; MAX_WP=200; MAX_LOCAL_WP=30; MAX_OBJ=32; MAX_TRAJ=30;

% Bus 정의 목록
busList = {
  'EgoStateBus',      {{'Car_Fr1_tx','double',1},{'Car_Fr1_ty','double',1},{'Car_Fr1_rz','double',1},{'Ego_V','double',1}};
  'ParsedLaneBus',    {{'x1','double',283},{'y1','double',283}, ...
                       {'x2','double',275},{'y2','double',275}, ...
                       {'x3','double',266},{'y3','double',266}, ...
                       {'x4','double',34}, {'y4','double',34}, ...
                       {'x5','double',19}, {'y5','double',19}};
  'EgoBus',          {{'time','double',1},{'x','double',1},{'y','double',1},{'yaw','double',1},{'vx','double',1},{'vy','double',1},{'ax','double',1},{'ay','double',1},{'yaw_rate','double',1},{'lane_idx','int32',1},{'s','double',1},{'d','double',1},{'mission_s','double',1},{'road_heading','double',1},{'road_curvature','double',1},{'valid','boolean',1}};
  'TrafficBus',      {{'id','int32',MAX_OBJ},{'valid','boolean',MAX_OBJ},{'x','double',MAX_OBJ},{'y','double',MAX_OBJ},{'vx','double',MAX_OBJ},{'vy','double',MAX_OBJ},{'ax','double',MAX_OBJ},{'ay','double',MAX_OBJ},{'yaw','double',MAX_OBJ},{'yaw_rate','double',MAX_OBJ},{'length','double',MAX_OBJ},{'width','double',MAX_OBJ}};
  'LaneMapBus',      {{'lane_x','double',[MAX_LANE MAX_WP]},{'lane_y','double',[MAX_LANE MAX_WP]},{'lane_yaw','double',[MAX_LANE MAX_WP]},{'lane_curvature','double',[MAX_LANE MAX_WP]},{'lane_valid','boolean',[MAX_LANE MAX_WP]},{'lane_wp_count','int32',MAX_LANE},{'tollgate_lane','int32',1},{'tollgate_pose','double',3},{'parking_pose','double',[2 3]},{'mission_zone','double',[4 2]}};
  'VehicleParamBus', {{'wheelbase','double',1},{'track_width','double',1},{'vehicle_length','double',1},{'vehicle_width','double',1},{'max_steer','double',1},{'max_steer_rate','double',1},{'max_accel','double',1},{'max_decel','double',1},{'min_speed','double',1},{'max_speed','double',1}};
  'ControllerParamBus',{{'LK_Kp_d','double',1},{'LK_Kd_d','double',1},{'LK_Kp_yaw','double',1},{'LK_lookahead_base','double',1},{'Long_Kp_v','double',1},{'Long_Ki_v','double',1},{'Long_Kd_v','double',1},{'TTC_min','double',1},{'front_gap_min','double',1},{'rear_gap_min','double',1},{'overtake_speed_margin','double',1},{'LC_time','double',1},{'LC_Kp_d','double',1},{'LC_Kd_d','double',1},{'LC_Kp_yaw','double',1},{'tollgate_prepare_distance','double',1},{'tollgate_target_speed','double',1},{'force_lane_keep_distance','double',1},{'parking_speed','double',1},{'parking_lookahead','double',1},{'PK_Kp_yaw','double',1},{'PK_Kd_yaw','double',1},{'PK_Kp_pos','double',1},{'goal_pos_tol','double',1},{'goal_yaw_tol','double',1}};
  'LocalLaneBus',    {{'local_x','double',[MAX_LANE MAX_LOCAL_WP]},{'local_y','double',[MAX_LANE MAX_LOCAL_WP]},{'local_yaw','double',[MAX_LANE MAX_LOCAL_WP]},{'local_curv','double',[MAX_LANE MAX_LOCAL_WP]},{'local_valid','boolean',[MAX_LANE MAX_LOCAL_WP]}};
  'LaneRiskBus',     {{'ttc','double',MAX_LANE},{'front_gap','double',MAX_LANE},{'rear_gap','double',MAX_LANE},{'front_rel_v','double',MAX_LANE},{'rear_rel_v','double',MAX_LANE},{'side_gap','double',MAX_LANE},{'lane_blocked','boolean',MAX_LANE},{'lane_cost','double',MAX_LANE},{'cut_in_risk','double',MAX_LANE},{'boundary_risk','double',MAX_LANE},{'safe_left','boolean',1},{'safe_right','boolean',1}};
  'MissionBus',      {{'mode','int32',1},{'target_lane','int32',1},{'target_speed','double',1},{'target_d','double',1},{'overtake_enable','boolean',1},{'lanechange_enable','boolean',1},{'tollgate_enable','boolean',1},{'parking_enable','boolean',1},{'emergency_stop','boolean',1},{'mission_done','boolean',1}};
  'CommandBus',      {{'valid','boolean',1},{'priority','int32',1},{'mode','int32',1},{'front_tire_angle','double',1},{'desired_ax','double',1},{'target_speed','double',1},{'target_lane','int32',1},{'done','boolean',1}};
  'StatusBus',       {{'valid','boolean',1},{'state','int32',1},{'error_code','int32',1},{'progress','double',1},{'target_lane','int32',1},{'lat_error','double',1},{'heading_error','double',1},{'front_gap','double',1},{'ttc','double',1},{'done','boolean',1}};
  'LC_RequestBus',   {{'valid','boolean',1},{'priority','int32',1},{'target_lane','int32',1},{'target_d','double',1},{'target_speed','double',1},{'forced','boolean',1},{'cancel','boolean',1},{'source','int32',1}};
  'OT_RequestBus',   {{'valid','boolean',1},{'priority','int32',1},{'request_lane_change','boolean',1},{'target_lane','int32',1},{'target_speed','double',1},{'return_lane','int32',1},{'abort','boolean',1},{'done','boolean',1}};
  'TG_RequestBus',   {{'valid','boolean',1},{'priority','int32',1},{'target_lane','int32',1},{'target_speed','double',1},{'prepare_distance','double',1},{'force_lane_keep','boolean',1},{'abort','boolean',1},{'done','boolean',1}};
  'TrajectoryBus',   {{'valid','boolean',MAX_TRAJ},{'x','double',MAX_TRAJ},{'y','double',MAX_TRAJ},{'yaw','double',MAX_TRAJ},{'curvature','double',MAX_TRAJ},{'v_ref','double',MAX_TRAJ},{'ax_ref','double',MAX_TRAJ},{'d_ref','double',MAX_TRAJ}};
};

for i = 1:size(busList,1)
    assignin('base', busList{i,1}, buildBus(busList{i,2}));
    fprintf('  %-22s : %d fields\n', busList{i,1}, length(busList{i,2}));
end
nFlat = size(busList,1);

% ----------------------------------------------------------------
% TrafficObjBus : 차량 1대 (T16~T28 각각에 공통 적용)
%   CarMaker 출력 기준 필드:
%     x, y          - 위치 (Tr.T##.Fr1.x / Fr1.y)
%     vx, vy        - 속도 (Tr.T##.Fr1.vx / vy)
%     ax, ay        - 가속도
%     yaw           - 차량 heading (Fr1.rz)
%     yaw_rate      - 요레이트 (rzv)
%     boundary      - [left, right] 차선 경계 거리 (2x1)
% ----------------------------------------------------------------
TrafficObjBus = buildBus({ ...
    {'x',        'double', 1}, ...
    {'y',        'double', 1}, ...
    {'vx',       'double', 1}, ...
    {'vy',       'double', 1}, ...
    {'ax',       'double', 1}, ...
    {'ay',       'double', 1}, ...
    {'yaw',      'double', 1}, ...
    {'boundary', 'double', 2}, ...
    {'yaw_rate', 'double', 1}  ...
});
assignin('base', 'TrafficObjBus', TrafficObjBus);
fprintf('  %-22s : 9 fields\n', 'TrafficObjBus');

% ----------------------------------------------------------------
% AllTrafficBus : T16~T28 (13대) 를 nested bus로 통합
%   접근 예시 : traffic.t16.x, traffic.t20.vx
% ----------------------------------------------------------------
TRAFFIC_IDS = 16:28;   % CarMaker Traffic Object 번호
e = Simulink.BusElement.empty;
for k = TRAFFIC_IDS
    el = Simulink.BusElement;
    el.Name       = sprintf('t%02d', k);   % t16, t17, ... t28
    el.DataType   = 'Bus: TrafficObjBus';
    el.Dimensions = 1;
    el.Complexity = 'real';
    el.SampleTime = -1;
    e(end+1) = el; %#ok<AGROW>
end
AllTrafficBus = Simulink.Bus;
AllTrafficBus.Elements = e;
assignin('base', 'AllTrafficBus', AllTrafficBus);
fprintf('  %-22s : %d fields (nested TrafficObjBus x%d)\n', ...
        'AllTrafficBus', length(TRAFFIC_IDS), length(TRAFFIC_IDS));

fprintf('\n✅ %d개 Bus 객체 등록 완료 (flat: %d, traffic: 2)\n', ...
        nFlat + 2, nFlat);
end

function bus = buildBus(fields)
    e = Simulink.BusElement.empty;
    for i = 1:length(fields)
        el = Simulink.BusElement;
        el.Name = fields{i}{1};
        el.DataType = fields{i}{2};
        el.Dimensions = fields{i}{3};
        el.Complexity = 'real';
        el.SampleTime = -1;
        e(end+1) = el; %#ok<AGROW>
    end
    bus = Simulink.Bus; bus.Elements = e;
end
