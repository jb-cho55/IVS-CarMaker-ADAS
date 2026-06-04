function [ego_lane_id, ego_cross_track_err, ego_head_err, ego_max_curvature] = Ego_Detector(ego_state, lanes, target_lane_fb, park_cmd)
%#codegen

    % ============================================================
    % 1. EgoStateBus unpack
    % ============================================================
    ego_x   = ego_state.Car_Fr1_tx;
    ego_y   = ego_state.Car_Fr1_ty;
    ego_yaw = ego_state.Car_Fr1_rz;

    % ============================================================
    % 2. ParsedLaneBus unpack - 5차선
    % ============================================================
    x1 = lanes.x1;  y1 = lanes.y1;
    x2 = lanes.x2;  y2 = lanes.y2;
    x3 = lanes.x3;  y3 = lanes.y3;
    x4 = lanes.x4;  y4 = lanes.y4;
    x5 = lanes.x5;  y5 = lanes.y5;

    % lane5 좌측 보정
    L5_OFFSET = 1.0;
    x5 = x5 + L5_OFFSET;

    % ============================================================
    % 3. 현재 ego가 가장 가까운 차선 판단
    % ============================================================
    d1 = nearest_d2(x1, y1, ego_x, ego_y);
    d2 = nearest_d2(x2, y2, ego_x, ego_y);
    d3 = nearest_d2(x3, y3, ego_x, ego_y);
    d4 = nearest_d2(x4, y4, ego_x, ego_y);
    d5 = nearest_d2(x5, y5, ego_x, ego_y);

    ego_lane_id = int32(1);
    dmin = d1;

    if d2 < dmin
        dmin = d2;
        ego_lane_id = int32(2);
    end

    if d3 < dmin
        dmin = d3;
        ego_lane_id = int32(3);
    end

    if d4 < dmin
        dmin = d4;
        ego_lane_id = int32(4);
    end

    if d5 < dmin
        ego_lane_id = int32(5);
    end

    % ============================================================
    % 4. target_lane 보호
    % ============================================================
    target_lane = int32(target_lane_fb);

    if target_lane < int32(1)
        target_lane = int32(1);
    elseif target_lane > int32(5)
        target_lane = int32(5);
    end

    % ============================================================
    % 5. 주차 진입 구간에서는 preview step 축소
    % ============================================================
    if park_cmd == int32(1) && ...
       (target_lane == int32(4) || target_lane == int32(5))
        LOOK_STEP = int32(2);
    else
        LOOK_STEP = int32(5);
    end

    % ============================================================
    % 6. target lane 기준 오차 / 곡률 계산
    % ============================================================
    if target_lane == int32(1)

        [ego_cross_track_err, ego_head_err, ego_max_curvature] = process_lane_closed( ...
            x1, y1, ego_x, ego_y, ego_yaw, LOOK_STEP);

    elseif target_lane == int32(2)

        [ego_cross_track_err, ego_head_err, ego_max_curvature] = process_lane_closed( ...
            x2, y2, ego_x, ego_y, ego_yaw, LOOK_STEP);

    elseif target_lane == int32(3)

        [ego_cross_track_err, ego_head_err, ego_max_curvature] = process_lane_closed( ...
            x3, y3, ego_x, ego_y, ego_yaw, LOOK_STEP);

    elseif target_lane == int32(4)

        [ego_cross_track_err, ego_head_err, ego_max_curvature] = process_lane_open( ...
            x4, y4, ego_x, ego_y, ego_yaw, LOOK_STEP);

    else

        [ego_cross_track_err, ego_head_err, ego_max_curvature] = process_lane_open( ...
            x5, y5, ego_x, ego_y, ego_yaw, LOOK_STEP);

    end

    % ============================================================
    % 7. [추가] 주차 진입차선(4/5)에서 우측차선에 딱붙어 진입
    %    controller가 cross_track_err->0 추종하므로, err에 offset을 더하면
    %    ego가 차선중심에서 그 반대방향으로 hold됨 (= 우측 hugging).
    %    부호/크기는 테스트로 확정.
    % ============================================================
    HUG_RIGHT_OFFSET = 1.2;   % [튜닝] 우측 오프셋[m]
    if park_cmd == int32(1) && (target_lane == int32(4) || target_lane == int32(5))
        ego_cross_track_err = ego_cross_track_err + HUG_RIGHT_OFFSET;
    end

end


function d2 = nearest_d2(xa, ya, ex, ey)
%#codegen

    d2 = 1e12;
    N  = int32(numel(xa));

    for i = 1:N
        dxk = xa(i) - ex;
        dyk = ya(i) - ey;
        v   = dxk*dxk + dyk*dyk;

        if v < d2
            d2 = v;
        end
    end

end


function [ego_cross_track_err, ego_head_err, ego_max_curvature] = process_lane_open(xa, ya, ex, ey, eyaw, LOOK_STEP)
%#codegen

    N = int32(numel(xa));

    best_i  = int32(1);
    best_d2 = 1e12;

    for i = 1:N
        dxk = xa(i) - ex;
        dyk = ya(i) - ey;
        v   = dxk*dxk + dyk*dyk;

        if v < best_d2
            best_d2 = v;
            best_i  = i;
        end
    end

    look_i = best_i + LOOK_STEP;

    if look_i > N
        look_i = N;
    end

    if look_i > best_i

        path_yaw = atan2( ...
            ya(look_i) - ya(best_i), ...
            xa(look_i) - xa(best_i));

    elseif best_i > int32(1)

        path_yaw = atan2( ...
            ya(best_i) - ya(best_i - int32(1)), ...
            xa(best_i) - xa(best_i - int32(1)));

    else

        path_yaw = eyaw;

    end

    dxr = ex - xa(best_i);
    dyr = ey - ya(best_i);

    ego_cross_track_err = -sin(path_yaw) * dxr + cos(path_yaw) * dyr;

    ego_head_err = wrap_pi(path_yaw - eyaw);

    ego_max_curvature = 0.0;
    STEP = int32(5);

    for kk = 0:4
        base_i = best_i + int32(kk) * STEP;

        i1 = base_i;
        i2 = base_i + STEP;
        i3 = base_i + int32(2) * STEP;

        if i1 > N
            i1 = N;
        end

        if i2 > N
            i2 = N;
        end

        if i3 > N
            i3 = N;
        end

        if i1 < i2 && i2 < i3

            yaw1 = atan2( ...
                ya(i2) - ya(i1), ...
                xa(i2) - xa(i1));

            yaw2 = atan2( ...
                ya(i3) - ya(i2), ...
                xa(i3) - xa(i2));

            dyaw = wrap_pi(yaw2 - yaw1);

            ds = sqrt( ...
                (xa(i3) - xa(i1))^2 + ...
                (ya(i3) - ya(i1))^2);

            if ds > 0.1
                kappa = abs(dyaw) / ds;

                if kappa > ego_max_curvature
                    ego_max_curvature = kappa;
                end
            end
        end
    end

end


function [ego_cross_track_err, ego_head_err, ego_max_curvature] = process_lane_closed(xa, ya, ex, ey, eyaw, LOOK_STEP)
%#codegen

    N = int32(numel(xa));

    best_i  = int32(1);
    best_d2 = 1e12;

    for i = 1:N
        dxk = xa(i) - ex;
        dyk = ya(i) - ey;
        v   = dxk*dxk + dyk*dyk;

        if v < best_d2
            best_d2 = v;
            best_i  = i;
        end
    end

    look_i = wrap_idx(best_i + LOOK_STEP, N);

    path_yaw = atan2( ...
        ya(look_i) - ya(best_i), ...
        xa(look_i) - xa(best_i));

    dxr = ex - xa(best_i);
    dyr = ey - ya(best_i);

    ego_cross_track_err = -sin(path_yaw) * dxr + cos(path_yaw) * dyr;

    ego_head_err = wrap_pi(path_yaw - eyaw);

    ego_max_curvature = 0.0;
    STEP = int32(5);

    for kk = 0:4
        base_i = wrap_idx(best_i + int32(kk) * STEP, N);

        i1 = base_i;
        i2 = wrap_idx(base_i + STEP, N);
        i3 = wrap_idx(base_i + int32(2) * STEP, N);

        yaw1 = atan2( ...
            ya(i2) - ya(i1), ...
            xa(i2) - xa(i1));

        yaw2 = atan2( ...
            ya(i3) - ya(i2), ...
            xa(i3) - xa(i2));

        dyaw = wrap_pi(yaw2 - yaw1);

        ds = sqrt( ...
            (xa(i3) - xa(i1))^2 + ...
            (ya(i3) - ya(i1))^2);

        if ds > 0.1
            kappa = abs(dyaw) / ds;

            if kappa > ego_max_curvature
                ego_max_curvature = kappa;
            end
        end
    end

end


function idx = wrap_idx(idx_in, N)
%#codegen
    idx = idx_in;

    while idx > N
        idx = idx - N;
    end

    while idx < int32(1)
        idx = idx + N;
    end

end


function a = wrap_pi(a)
%#codegen

    while a > pi
        a = a - 2.0*pi;
    end

    while a < -pi
        a = a + 2.0*pi;
    end

end