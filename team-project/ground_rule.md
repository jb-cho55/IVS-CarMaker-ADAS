# Ground Rule

## 1. Branch Strategy

### Making Branch

- Always create an Issue first before starting work.
- Create a branch based on the Issue number.
- Never commit directly to the `main` branch.
- After finishing the issue, create a Pull Request(PR).

### Branch Naming Convention

Format:

```bash
#xx-solving_prob-name
```

Example:

```bash
#01-adding_assets-Hyun
#12-fixing_login_error-John
#25-refactoring_auth-module
```

---

## 2. Git Commit Convention

### Commit Message Format

```bash
type: description
```

### Commit Types

| Type | Description |
|------|-------------|
| feat | 새로운 기능 추가 ✨ |
| fix | 버그 수정 🐛 |
| docs | 문서 수정 📄 |
| style | 코드 포맷팅 등 스타일 변경 🎨 |
| refactor | 기능 변화 없는 코드 리팩토링 🛠️ |
| perf | 성능 개선 ⚡ |
| test | 테스트 추가 또는 수정 🧪 |
| chore | 기타 작업 (예: 의존성 업데이트) |

---

## 3. Commit Examples

```bash
feat: add login API
fix: resolve token validation bug
docs: update README installation guide
style: format code with prettier
refactor: simplify authentication logic
perf: optimize image loading speed
test: add unit test for user service
chore: update npm dependencies
```

---

## 4. Pull Request Rule

- PR title should clearly describe the work.
- Link the related Issue in the PR description.
- Ensure all tests pass before merging.
- Request code review before merge.

Example:

```md
## Related Issue
#12

## What Changed
- Added login validation
- Fixed token refresh bug

## Checklist
- [x] Test completed
- [x] No direct commit to main
- [x] PR reviewed
```
