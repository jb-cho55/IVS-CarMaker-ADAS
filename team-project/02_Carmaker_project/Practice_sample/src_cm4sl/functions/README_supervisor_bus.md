# Supervisor + Bus 구조 — 시나리오 Owner 중심

## .m 파일 (시나리오 owner 중심 이름)

| 파일 | 시그니처 | Owner | 시나리오 |
|---|---|---|---|
| dev_f_supervisor.m | ctrl_bus = dev_f_supervisor(ego_v, ego_x, ego_y, ego_yaw, cte_raw, traffic_x, traffic_y, traffic_v, v_target_user) | Dev-F | Cross-scenario |
| dev_a_basic_driving.m | acc_cmd = dev_a_basic_driving(ctrl_bus, mode) | Dev-A | 1-1 기본 주행 |
| dev_b_overtaking.m | mode = dev_b_overtaking(ctrl_bus, target_lane) | Dev-B | 1-2 추월 |
| dev_c_tollgate.m | cte_offset = dev_c_tollgate(ctrl_bus, mode) | Dev-C | 1-3 톨게이트 |
| dev_d_parking.m | [target_lane, lane_cost] = dev_d_parking(ctrl_bus) | Dev-D | 2-1 주차 |
| dev_e_lateral.m | steer_cmd = dev_e_lateral(ctrl_bus, mode, cte) | Dev-E | Cross (모든) |

## Simulink 통합 상태

### 자동 완료 (코드로 처리)
- ✅ Lib_Supervisor SubSystem 추가
- ✅ Wrapper_supervisor_fcn 코드 (dev_f_supervisor 호출)
- ✅ Lib_Supervisor: 9 Inport (ego_v, ego_x, ..., v_target_user) + 1 Outport (ctrl_bus) + wiring 완료
- ✅ 5 wrapper 코드 새 함수 이름으로 갱신

### 사용자 수동 작업 필요
1. **Lib_Supervisor 외부 wiring** (9개 Inport에 신호 연결):
   - ego_v       ← Read[Car.v]
   - ego_x       ← Read[Car.Fr1.x]
   - ego_y       ← Read[Car.Fr1.y]
   - ego_yaw     ← Read[Car.Yaw]
   - cte_raw     ← Read[Road.DevDist]
   - traffic_x   ← Read[Traffic.T22.x] (또는 Const 0)
   - traffic_y   ← Read[Traffic.T22.y] (또는 Const 0)
   - traffic_v   ← Read[Traffic.T22.v] (또는 Const 0)
   - v_target_user ← Constant 20
2. **Lib_Supervisor 출력**: ctrl_bus → Goto[CTRL_BUS]
3. **5 Lib의 Inport 단순화**: 옛 다중 신호 Inport 삭제, ctrl_bus 단일 Inport로
4. **Bus Object 정의** (권장):
   ```matlab
   sample = dev_f_supervisor(20,100,-1.5,0,0.1,130,-1.5,15,20);
   Simulink.Bus.createObject(sample);
   ```
5. **Update Diagram (Ctrl+D)** + **Save (Ctrl+S)**

## 백업
- functions_backup_before_4scn/  (placeholder 시절)
- functions_backup_before_rename/ (이름 변경 직전)
