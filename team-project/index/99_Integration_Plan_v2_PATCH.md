# Patch v2 — 99_Integration_Plan.md

> 원본의 §1 Golden Rules와 §3 Checkpoint 검증에 항목 추가.

---

## ▶ 변경 #1: §1 Golden Rules — 7번째 항목 추가

기존 6개 Golden Rules 끝에 다음을 추가:

```
7. **Goto 태그 정규화** — Day1~3 모델의 오타 `Ego_Gloabl_Y`는
   00_InputAdapter에서 반드시 `Ego_Global_Y`로 변환. Feature 모듈은
   표준 이름만 사용. Day1~3 원본을 그대로 가져다 쓸 때 주의.
```

---

## ▶ 변경 #2: §3 Checkpoint — DVA 자동 스캔 검증 추가

### CP1 (Skeleton Build) — 다음 항목 추가
```
- [ ] DVA 자동 스캔: Adapter 외부에 Read CM Dict / Write CM Dict 없음 (0개)
- [ ] Goto 태그 표준화: Ego_Global_Y 사용 (Ego_Gloabl_Y 오타 0개)
```

### CP2~CP6 — 매 단계 다음 항목 추가
```
- [ ] DVA 자동 스캔 결과가 CP1과 동일 (Adapter 외 새 Read/Write 없음)
- [ ] 신규 Goto 태그 추가 시 04 §11에 등록 + InputAdapter 안에서 발행
```

---

## ▶ 변경 #3: 새 부록 — DVA 자동 스캔 스크립트

통합 시 매 단계 실행할 검증 스크립트 (참고용 sample):

### `scripts/check_dva_compliance.m`
```matlab
function check_dva_compliance(model)
  if nargin < 1, model = 'Final_Project'; end
  load_system(model);

  %% 1) Adapter 외부의 Read CM Dict 검출
  allReads = find_system(model, 'ReferenceBlock', 'CarMaker4SL/Read CM Dict');
  bad = {};
  for k = 1:length(allReads)
    p = allReads{k};
    if ~contains(p, '00_InputAdapter')
      bad{end+1,1} = p;
    end
  end
  if ~isempty(bad)
    fprintf('❌ Adapter 외부 Read CM Dict %d개:\\n', length(bad));
    for k = 1:length(bad), fprintf('   - %s\\n', bad{k}); end
  else
    fprintf('✅ Read CM Dict 모두 00_InputAdapter 안에 있음\\n');
  end

  %% 2) Write CM Dict도 동일하게 OutputAdapter 안에 있는지
  allWrites = find_system(model, 'ReferenceBlock', 'CarMaker4SL/Write CM Dict');
  bad2 = {};
  for k = 1:length(allWrites)
    p = allWrites{k};
    if ~contains(p, '10_OutputAdapter')
      bad2{end+1,1} = p;
    end
  end
  if ~isempty(bad2)
    fprintf('❌ Adapter 외부 Write CM Dict %d개:\\n', length(bad2));
    for k = 1:length(bad2), fprintf('   - %s\\n', bad2{k}); end
  else
    fprintf('✅ Write CM Dict 모두 10_OutputAdapter 안에 있음\\n');
  end

  %% 3) Goto 태그 오타 검출
  allGotos = find_system(model, 'BlockType', 'Goto');
  typoTags = {};
  for k = 1:length(allGotos)
    tag = get_param(allGotos{k}, 'GotoTag');
    if strcmp(tag, 'Ego_Gloabl_Y') || strcmp(tag, 'Ego_Velocity')
      typoTags{end+1,1} = sprintf('%s (tag: %s)', allGotos{k}, tag);
    end
  end
  if ~isempty(typoTags)
    fprintf('⚠️ 비표준 Goto 태그 %d개:\\n', length(typoTags));
    for k = 1:length(typoTags), fprintf('   - %s\\n', typoTags{k}); end
  else
    fprintf('✅ Goto 태그 표준화 완료\\n');
  end
end
```

---

## ▶ 변경 #4: Final Project Traffic 매핑 명시

기존 문서에 명시 안 됨. CP1 Skeleton 단계에서 다음 표 InputAdapter 안에 주석으로 포함:

### Final Project 시나리오 Traffic ID 매핑

| CarMaker 실제 변수 | Goto 태그 (표준) | TrafficBus index |
|---|---|---|
| `Traffic.T22.rzv` | `Traffic00_YawRate` | `yaw_rate(1)` |
| `Traffic.T23.rzv` | `Traffic01_YawRate` | `yaw_rate(2)` |
| `Traffic.T24.rzv` | `Traffic02_YawRate` | `yaw_rate(3)` |
| `Traffic.T25.rzv` ~ `T28.rzv` | `Traffic03_YawRate` ~ `Traffic06_YawRate` | `yaw_rate(4..7)` |

> ⚠️ 주의: 실제 CarMaker scenario의 T22~T28이 Day6와 다르게 매핑됨.
> Day6 코드는 T00~T02 사용하므로, Final Project에서 그대로 가져오면 신호 0이 들어옴.
