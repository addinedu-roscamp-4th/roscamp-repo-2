# 📖 GitHub 협업 가이드 및 명령어 정리

RoboDine 프로젝트 협업 시 팀원들이 GitHub를 효과적으로 사용하기 위한 가이드입니다.

---

## 🌿 GitHub 협업 기본 흐름

### 1. 최신 코드 유지 (pull)

```bash
git checkout develop
git pull origin develop
```

### 2. 기능 개발 시작 (feature branch 생성)

```bash
git checkout -b feature/<기능명>
# 예시
git checkout -b feature/ros-service-integration
```

### 3. 개발 완료 후 변경사항 추가 및 커밋(commit)

```bash
git add .
git commit -m "feat: ROS 서비스 통합 기능 추가 (#12)"
```

### 4. 원격 저장소에 push

```bash
git push origin feature/<기능명>
# 예시
git push origin feature/ros-service-integration
```

### 5. Pull Request 생성 및 코드 리뷰 요청

- GitHub에서 Pull Request를 생성합니다.
- `develop` 브랜치로 병합 요청합니다.
- 최소 1명 이상의 팀원에게 코드 리뷰를 요청합니다.

---

## 📌 기본 GitHub 명령어 모음

| 명령어 | 설명 | 예시 |
|--------|------|------|
| `git clone` | 저장소 복제 | `git clone https://github.com/username/repo.git` |
| `git branch` | 브랜치 확인 | `git branch -a` |
| `git checkout` | 브랜치 전환 및 생성 | `git checkout -b feature/login` |
| `git add` | 변경된 파일 추가 | `git add .` |
| `git commit` | 변경 사항 저장 | `git commit -m "feat: 로그인 추가"` |
| `git push` | 변경 사항 원격 저장소로 push | `git push origin 브랜치이름` |
| `git pull` | 원격 저장소 최신 코드 가져오기 | `git pull origin develop` |
| `git merge` | 브랜치 병합 | `git merge feature/login` |
| `git status` | 현재 상태 확인 | `git status` |

---

## ✅ 협업 규칙 및 커밋 메시지 예시

### 브랜치 전략

- `main`: 안정화된 배포 상태 유지
- `develop`: 개발 통합 브랜치
- `feature/<기능명>`: 개별 기능 개발 브랜치

### 커밋 메시지 규칙

```bash
feat: 신규 기능 추가 (#이슈번호)
fix: 버그 수정 (#이슈번호)
docs: 문서화 (#이슈번호)
refactor: 코드 리팩토링 (#이슈번호)
```

예시:

```bash
git commit -m "feat: 주문 처리 REST API 구현 (#32)"
```

---

## 🚀 GitHub Actions CI/CD 확인하기

- GitHub Actions는 코드 푸시 또는 PR 생성 시 자동으로 실행됩니다.
- 테스트가 실패하면 병합되지 않으므로, 코드 푸시 전 로컬에서 꼭 테스트해주세요.

---

이 문서를 참조하여 원활한 협업과 효율적인 코드 관리를 진행해 주세요! 🚀✨
