# desired_ax(AccelCtrl)를 이용한 종방향 제어 정리

> 출처: CarMaker Reference Manual 14.1.2 (§7.4 ACC Controller, §26.6.2 UAQ, 기어/DVA 관련 절).
> "📘 매뉴얼" = 매뉴얼에 명시된 사실 / "🧪 실측" = 우리 프로젝트에서 확인한 경험.

---

## 1. AccelCtrl 이란 (📘 §7.4.2)
- **AccelCtrl (Acceleration Control)** = CarMaker의 VehicleControl 모델. **원하는 종방향 가속도(desired longitudinal acceleration)를 입력받아 가스/브레이크 페달 위치를 만들어내는 PI 제어기.**
- 즉 우리는 "가속도 명령(m/s²)"만 주면 되고, **페달(VC.Gas/VC.Brake)은 AccelCtrl이 알아서 계산**한다.
- 비활성 시(ACC off) 페달에 손대지 않음.

## 2. 셋업 (📘 §7.4)
차량 파일(예: `Data/Vehicle/Kia_EV6`)에 VehicleControl 모델로 AccelCtrl 지정:
```
VehicleControl.0.Kind = AccelCtrl 1
AccelCtrl.DesrAccelFunc = <ACC | DVA | User>
AccelCtrl.p = 0.001      # PI 비례 게인 (기본 0.001)
AccelCtrl.i = 1.0        # PI 적분 게인 (기본 1.0)
```

### `DesrAccelFunc` — desired_ax를 누가 계산하나 (📘 §7.4.3)
| 값 | 의미 |
|---|---|
| **ACC** (기본) | CarMaker 내장 ACC 컨트롤러가 desired accel 계산(선행차 거리/속도 추종) |
| **DVA** | **Direct Variable Access로 외부에서 `AccelCtrl.DesiredAx`에 직접 씀** (Simulink/IPGControl/User) |
| **User** | 사용자 C 함수가 계산 (`User.c`에서 `AccelCtrl.DesrAx = Ax;`, `Set_UserDesrAccelFunc()` 등록) |

- **모든 모드에서 `AccelCtrl.DesiredAx = NOTSET(-99999)`를 주면 가속도 제어 OFF** → 페달 제어가 DrivMan(주행 시나리오)로 넘어감. (📘 §7.4.3)

## 3. desired_ax 양 (quantity) (📘 §26.6.2)
| UAQ 이름 | C-Code | 단위 | 설명 |
|---|---|---|---|
| **`AccelCtrl.DesiredAx`** | `AccelCtrl.DesrAx` | **m/s²** | AccelCtrl이 추종할 목표 종방향 가속도. `-99999(NOTSET)` = 사용 안 함 |
- 이게 우리가 Simulink에서 **Write CM Dict**로 쓰는 바로 그 신호.

## 4. PI 제어기 + 신호 흐름 (📘 §7.4.2, Fig 7.3)
```
 DrivMan ─ DM.Gas/DM.Brake ┐
                           │   a_desr        a_actual(피드백)
 [ACC | DVA | User] ─ a_desr ─►  PI 제어기  ─► VC.Gas / VC.Brake ─► Vehicle(Powertrain, Brake)
                                (p,i)
```
- AccelCtrl은 **error = a_desr − a_actual** 로 PI 동작 → 페달 산출.
- 즉 desired_ax는 **"이만큼 가속/감속하라"는 목표**이고, AccelCtrl이 실제 가속도가 그에 맞도록 페달을 조절.

## 5. desired_ax 쓰는 법 — DVA (📘 §1.x, §7.4.3)
- **DVA(Direct Variable Access)** = 시뮬레이션 중 UAQ(여기선 `AccelCtrl.DesiredAx`)에 외부에서 값을 써넣는 것.
- 쓰는 경로: **Simulink "Write CM Dict" 블록**, IPGControl의 DVA 창, 또는 User.c.
- UAQ는 **사이클당 1회(보통 1ms) 갱신**됨. (📘 §powertrain DVA 노트)
- `TestRunEnd.DVA_ReleaseAll`로 테스트런 종료 시 DVA write 해제 여부 설정.

## 6. 방향(기어) = `DM.SelectorCtrl` (📘 §gear)
- desired_ax는 **가속도 크기/부호**만; **진행 방향은 기어(SelectorCtrl)로 결정.**
- `DM.SelectorCtrl` 값 (📘 P=-9, M=2 명시; 전체 enum은 `include/Vehicle.h`):

  | 값 | 위치 |
  |---|---|
  | **-9** | P (주차) |
  | **-1** | R (후진) |
  | **0** | N (중립) |
  | **1** | D (주행/전진) |
  | **2** | M (Manumatic, 수동단 지정) |
  | **3** | S (스포츠/2차 시프트맵) |

- **Manumatic**: `DM.SelectorCtrl=2`로 두고 `DM.GearNo`에 목표 단수 지정. (📘 §17574)
- 자동변속 P(-9)에선 parking lock torque 작동. (📘 §23504)

---

## 7. ⭐ 실전 메모 — 우리 프로젝트 (🧪 실측)
매뉴얼대로지만, 실제 적용에서 확인/주의할 점:

### (a) 부호 규약
- **`AccelCtrl.DesiredAx` 양수 = 가스(기어 진행방향으로 가속), 음수 = 브레이크.** 방향은 SelectorCtrl이 결정.
  - 전진(D)+양수 → 앞으로 가속 / 후진(R)+양수 → 뒤로 가속 / 음수 → (어느 기어든) 브레이크.
  - 🧪 후진에서 음수를 주면 그냥 브레이크라 **안 움직임**. 후진도 **양수**를 줘야 뒤로 감.

### (b) 후진 windup (★ 가장 중요한 함정)
- AccelCtrl PI는 **차체 기준 종가속도(a_actual)** 로 error를 닫는다. 가속도는 차체 x(앞)=+.
- **전진**: 가스 → a_actual 양수 → error=(a_desr+ − a_actual+) **줄어듦** → 안정/매끈.
- **후진**: 가스 → 뒤로 가속 → a_actual **음수** → error=(a_desr+ − a_actual−) **커짐** → **적분 windup → 급발진**.
- 결론: **AccelCtrl은 전진 전용에 가깝다.** 후진에서 desired_ax로 "일정속도 cruise"를 매끈하게 유지하는 것은 구조적으로 어려움:
  - 양수 지속 → windup 급발진
  - 0 → (드래그를 error로 읽어) 브레이크 → 정지
  - 그 사이를 오가며 제어 → **후진-정지-후진 펄스**(끊김)
- 실용 대응: 후진은 **짧게**(실제 주차는 2~3m). 짧은 후진은 "가속→감속" 한 번이라 펄스가 거의 안 보임. 긴 후진만 펄스가 도드라짐.

### (c) 우리 셋업의 DesrAccelFunc
- 🧪 현재 Kia_EV6는 `DesrAccelFunc = User`. 이 CM4SL 셋업에선 **User 모드에서 `AccelCtrl.DesiredAx` write가 구동에 반영됨**(실측). DVA로 바꿨더니 오히려 무반응이었음(환경/버전 특성으로 추정).
  - → 매뉴얼상 외부 write의 정석 모드는 **DVA**지만, 우리 환경에선 **User 유지**가 정답이었음. (변경 금지)
- p=0.001, **i=0.1**(기본 1.0에서 낮춤 — 급가속 억제용. 낮을수록 windup 느림).

### (d) 인터페이스 단일화
- 강사 지침: **VC.Gas/VC.Brake 직접 제어 금지** → `AccelCtrl.DesiredAx` + `DM.SelectorCtrl`만 사용.
- 따라서 후진 windup도 desired_ax 범위 안에서만 해결해야 함(페달 우회 불가).

---

## 8. 한 줄 요약
- **종방향**: `AccelCtrl.DesiredAx`(m/s², +가속/−브레이크) 하나만 주면 AccelCtrl PI가 페달 생성.
- **방향**: `DM.SelectorCtrl`(1=D / −1=R / −9=P).
- **함정**: AccelCtrl이 전진용이라 **후진은 windup**(양수 지속→급발진). 후진은 짧게/펄스 감안.

## 부록 A. 구조 그림 해석 (Fig 7.1 / 7.2 / 7.3, §7.2·§7.3)

### A-1. §7.2 VehicleControl 파이프라인 (Fig 7.1)
```
DrivMan ─(DM.Gas/Brake/Clutch/GearNo/Steer.Ang…)─► [VehicleControl] ─(VC.*)─► Vehicle(Steering,PowerTrain,Brake)
```
- VehicleControl은 **DrivMan 다음, Vehicle 이전**에 호출.
- 호출 직전 **DM.* 신호가 VC.* 로 복사(기본값)** → VehicleControl 모델이 VC.* 수정 → Vehicle 입력.
- **AccelCtrl이 이 VehicleControl 모델 중 하나** → VC.Gas/VC.Brake를 만든다. (우리가 페달 직접 안 써도 되는 이유)

### A-2. §7.3 모델 등록 (어떻게 AccelCtrl을 켜나)
- `VehicleControl.<i>.Kind = KindString Ver` (i=0..9, 최대 10슬롯 적층) 또는 `.FName=파일`.
- 파라미터는 **TestRun 먼저 → 없으면 vehicle 파일**.
- 라이브러리: **AccelCtrl**(종방향 PI), GenLongCtrl(AEB/FCW), GenLatCtrl(LKAS/LDW).
- 우리: `VehicleControl.0.Kind = AccelCtrl 1` (Kia_EV6 L624), 파라미터는 vehicle 파일에 직접.
- "+ ACC" = AccelCtrl 안에 ACC 서브컨트롤러 포함(우리 미사용). "1" = 버전.

### A-3. §7.4.1 ACC 모드 내부 (Fig 7.2) — 우리 미사용
- 내장 ACC(어댑티브 크루즈): 선행차 없으면 속도제어, 있으면 거리제어.
- `a_desr = k_v·Δv + k_d·Δd` → `[a_min,a_max]` 제한 → PI. (우리는 이 a_desr를 직접 주므로 ACC 안 씀)

### A-4. §7.4.2 AccelCtrl PI (Fig 7.3) — ★핵심
```
[ACC | DVA | User] ─ a_desr ─►  PI:  Δa = a_desr − a_actual  ─► VC.Gas / VC.Brake
```
- a_desr 소스 3종 중 우리는 DVA/User로 `AccelCtrl.DesiredAx`를 줌.
- **PI가 Δa로 페달 결정.** → a_desr를 **급변시키면 PI가 과반응(surge)**, 그래서 a_desr를 완만히 주는 게 유리.
- **후진 windup이 바로 이 Δa 루프**: 후진 가스 → a_actual(차체기준) 음수 → Δa 커짐 → 적분 폭주.

### A-5. §6.2 DrivMan 시작값
- `DrivMan.SpeedUnit`(kmh/ms), `DrivMan.Man.Start.Velocity`(초기속도). → 우리 TestRun이 `Start.Velocity=0`이라 정지 출발.

## 출처(매뉴얼 절)
- §7.4.1 ACC Controller Model
- §7.4.2 ACC Controller & VehicleControl AccelCtrl (Fig 7.3)
- §7.4.3 Model Parametrization (DesrAccelFunc, p/i, NOTSET)
- §7.4.4 / §26.6.2 User Accessible Quantities (AccelCtrl.DesiredAx ↔ DesrAx, m/s²)
- 기어: DM.SelectorCtrl(P=-9, M=2), DM.GearNo (Manumatic) — §gear/§17574/§23504
