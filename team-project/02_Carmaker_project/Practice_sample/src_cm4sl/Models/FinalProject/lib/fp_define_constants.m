function fp_define_constants()
%FP_DEFINE_CONSTANTS Final Project 전역 상수 base workspace 등록

% 크기 상수
assignin('base', 'MAX_LANE',     int32(4));
assignin('base', 'MAX_WP',       int32(200));
assignin('base', 'MAX_LOCAL_WP', int32(30));
assignin('base', 'MAX_OBJ',      int32(32));
assignin('base', 'MAX_TRAJ',     int32(30));

% 샘플 타임
assignin('base', 'TS_CONTROL',    0.01);
assignin('base', 'TS_PLANNING',   0.05);
assignin('base', 'TS_SUPERVISOR', 0.05);

% 모드 enum
assignin('base', 'MODE_IDLE',        int32(0));
assignin('base', 'MODE_LANE_KEEP',   int32(1));
assignin('base', 'MODE_OVERTAKE',    int32(2));
assignin('base', 'MODE_LANE_CHANGE', int32(3));
assignin('base', 'MODE_TOLLGATE',    int32(4));
assignin('base', 'MODE_PARKING',     int32(5));
assignin('base', 'MODE_EMERGENCY',   int32(9));

% 우선순위
assignin('base', 'PRIO_LANE_KEEP',   int32(1));
assignin('base', 'PRIO_TOLLGATE',    int32(2));
assignin('base', 'PRIO_LANE_CHANGE', int32(3));
assignin('base', 'PRIO_PARKING',     int32(4));
assignin('base', 'PRIO_EMERGENCY',   int32(5));

% Vehicle Params (Kia EV6)
VP = struct();
VP.wheelbase=2.7; VP.track_width=1.62;
VP.vehicle_length=4.68; VP.vehicle_width=1.88;
VP.max_steer=pi/6; VP.max_steer_rate=pi/2;
VP.max_accel=3.0; VP.max_decel=6.0;
VP.min_speed=0.0; VP.max_speed=33.33;
assignin('base', 'VP', VP);

% Controller Params (초기 게인)
CP = struct();
CP.LK_Kp_d=0.5; CP.LK_Kd_d=0.1; CP.LK_Kp_yaw=0.8; CP.LK_lookahead_base=8.0;
CP.Long_Kp_v=0.8; CP.Long_Ki_v=0.05; CP.Long_Kd_v=0.1;
CP.TTC_min=3.0; CP.front_gap_min=15.0; CP.rear_gap_min=10.0; CP.overtake_speed_margin=5.0;
CP.LC_time=3.0; CP.LC_Kp_d=0.5; CP.LC_Kd_d=0.1; CP.LC_Kp_yaw=1.0;
CP.tollgate_prepare_distance=100.0; CP.tollgate_target_speed=8.33; CP.force_lane_keep_distance=30.0;
CP.parking_speed=1.5; CP.parking_lookahead=2.0;
CP.PK_Kp_yaw=1.5; CP.PK_Kd_yaw=0.2; CP.PK_Kp_pos=0.8;
CP.goal_pos_tol=0.3; CP.goal_yaw_tol=0.087;
assignin('base', 'CP', CP);

fprintf('✅ 상수 등록 (MAX_*, TS_*, MODE_*, PRIO_*, VP, CP)\n');
end
