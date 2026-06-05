# MCP로 CarMaker 제어하기 (CM4SL 기반)

이 문서는 Claude(또는 MCP 클라이언트)가 **MATLAB MCP 서버를 통해 CarMaker를 구동·제어·분석**하는 방법을 정리한 것입니다.
이번 day7 주차 프로젝트에서 실제로 사용한 절차/명령 기준입니다.

---

## 0. 핵심 개념
- **CarMaker 전용 MCP는 없습니다.** 제어는 **MATLAB MCP**를 경유합니다.
- 체인:
  ```
  Claude ──(mcp__matlab__evaluate_matlab_code)──▶ MATLAB / Simulink + CM4SL
                                                  └──(cmguicmd: Tcl)──▶ CarMaker GUI/엔진
  ```
- CarMaker는 **CM4SL(CarMaker for Simulink)** 로 Simulink 모델(`generic_IVS`)과 결합.
- 시뮬레이션 구동은 **Simulink 쪽**(`set_param ... 'start'`)에서 하고, 설정/제어는 **GUI Tcl**(`cmguicmd`)로 보냅니다.

---

## 1. 사용하는 MCP 도구 (matlab 서버)
| 도구 | 용도 |
|------|------|
| `mcp__matlab__evaluate_matlab_code` | 인라인 MATLAB 실행 (주력) |
| `mcp__matlab__run_matlab_file` | .m 스크립트 파일 실행 |
| `mcp__matlab__check_matlab_code` | 정적 분석(문법 점검) |
| `mcp__matlab__detect_matlab_toolboxes` | 설치 버전/툴박스 확인 |

> 전제: MATLAB 데스크톱이 열려 있고 matlab MCP 서버가 연결돼 있어야 함. CarMaker(예: `C:\IPG\carmaker\win64-14.1.2`) 설치 + CM4SL 프로젝트 존재.

---

## 2. 단계별 제어 (실제 명령)

### (A) 환경 설정 — 세션당 1회
```matlab
cd('C:\Users\User\Desktop\ple\02_Carmaker_project\Practice_sample\src_cm4sl');
cmenv;          % CarMaker/CM4SL 경로 로드 + cminit (Initialize CarMaker for Simulink)
```

### (B) 모델 열기 + GUI 실행/연결
```matlab
open_system('generic_IVS');     % CM4SL 모델
CM_Simulink;                    % CarMaker for Simulink GUI 기동 + 활성모델 연결
% 연결 확인 (status 0 = OK):
[res, st] = cmguicmd('SimStatus', 5000);
```

### (C) GUI 명령 — `cmguicmd(<Tcl>, timeout_ms)`
```matlab
cmguicmd('LoadTestRun day7_park_entry', 10000);          % TestRun 로드
cmguicmd('SaveMode save', 3000);                          % 결과 .erg 저장 ON (실행 전에!)
cmguicmd('OutQuantsAdd {Time Car.Fr1.tx Car.vx ...}', 5000); % .erg 기록 양 추가
cmguicmd('QuantSubscribe {Time Car.Fr1.tx Car.Fr1.ty Car.Fr1.rz Car.vx DM.SelectorCtrl}', 3000);
```
구독한 양 읽기(`$Qu`):
```matlab
[r, st] = cmguicmd('format "t=%.2f x=%.2f v=%.2f" $Qu(Time) $Qu(Car.Fr1.tx) $Qu(Car.vx)', 3000);
v = sscanf(r, '%*[^0-9-]%f ...');   % 또는 직접 파싱
```
자주 쓰는 Tcl: `LoadTestRun`, `StartSim`/`StopSim`, `WaitForStatus running|idle`, `SimStatus`,
`SaveMode {save|collect|hist_10s|off}`, `SaveStart`/`SaveStop`, `OutQuantsAdd/Del`, `QuantSubscribe`, `DVAWrite`.

### (D) 시뮬레이션 구동 — Simulink에서
```matlab
set_param('generic_IVS', 'SimulationCommand', 'start');
get_param('generic_IVS', 'SimulationStatus')    % 'running' / 'stopped'
set_param('generic_IVS', 'SimulationCommand', 'stop');
```
> StopTime=inf인 경우가 많아 종료 조건을 직접 감지(폴링)해서 StopSim 해야 함.

### (E) 결과 읽기 / 분석
- **실시간**: 위 `cmguicmd + $Qu` 를 pause로 폴링.
- **종료 후 정밀(.erg)**:
  ```matlab
  R = cmread('SimOutput\<host>\<date>\<run>.erg');   % R.<채널>.data, .unit
  t = R.Time.data;  x = R.Car_Fr1_tx.data;           % 점(.)→밑줄(_) 로 채널명
  ```
- **모델 logsout(To Workspace)**:
  ```matlab
  ls = evalin('base','logsout');  ls.getElement('ego_v').Values.Data
  ```

### (F) 모델/제어로직 수정 (개선 반복)
```matlab
% MATLAB Function 블록 = Stateflow EMChart 의 .Script
rt = sfroot; ch = rt.find('-isa','Stateflow.EMChart');
% 경로로 대상 찾기: ch(i).Path == 'generic_IVS/.../MATLAB Function'
ch(i).Script = newCode;                 % 소스 교체 (strrep/regexprep로 부분수정)
checkcode('Parking.m','-string')        % 문법 점검
% DVA 쓰기 블록(Write CM Dict) 마스크/배선:
get_param(blk,'MaskValues'); set_param(blk,'MaskValues',{'AccelCtrl.DesiredAx','off'});
add_line(sc,'MATLAB Function/1','Write CM Dict2/1','autorouting','on');
save_system('generic_IVS','SaveDirtyReferencedModels','on');  % 참조 서브시스템까지 저장
```

---

## 3. 표준 1-사이클 워크플로우
```
cmenv
 → open_system + CM_Simulink            (최초 연결)
 → LoadTestRun → SaveMode save → OutQuantsAdd → QuantSubscribe
 → set_param('...','start')
 → (pause 폴링으로 진행/종료 감지)
 → StopSim / set_param('...','stop')
 → cmread(.erg) 로 정밀 분석
 → (필요시 EMChart.Script 수정) → 다시 실행   ← 반복
```

---

## 4. 실전 함정 (꼭 기억)
1. **`$Qu(x)` 읽기 전에 반드시 `QuantSubscribe`** 해야 함. 안 하면 "no such element".
2. **시뮬 실행 중엔 차트가 잠김** → `chart.Script` 수정 전에 **반드시 StopSim/stop**. ("잠겨 있으므로 수정 불가" 에러).
3. **CM4SL 시작은 Simulink(`set_param start`)** 가 안전. GUI `StartSim`도 되지만 Simulink 결합 상태여야.
4. **.erg 저장은 `SaveMode save` 를 실행 전에** 설정 (그 후 자동 저장, 종료 시 SimOutput에 기록).
5. `cmguicmd` 안에서 **MATLAB으로 되묻는 Tcl은 데드락** 위험 (cmguicmd 실행 중 MATLAB 차단됨).
6. `get_param(...){1}` 같은 **즉시 셀 인덱싱 불가** → 임시변수에 받아서 인덱싱.
7. 채널명: CarMaker `Car.Fr1.tx` → cmread 구조체 필드 `Car_Fr1_tx`.
8. MCP **Bash로 `claude remote-control` 등 장기 서버 실행은 블로킹/별도세션** → 부적합. (원격제어는 세션 내 `/remote-control` 사용)

---

## 5. 이 프로젝트의 제어 인터페이스 (참고)
- 입력(Read CM Dict): `Car.Fr1.tx/ty/rz`, `Car.vx`
- 출력(Write CM Dict): `AccelCtrl.DesiredAx`(종방향), `Car.CFL/CFR.rz_ext`(조향), `DM.SelectorCtrl`(기어)
- 제어로직: 3개 self-contained MATLAB Function 블록(Parking / generate_map_ / add_obstacle_),
  참조 서브시스템 파일 `src_cm4sl/Models/Day4_5/Day4_5_Scenario_1.slx` (이식 가능).
