# ROST_BOT
본 파일은 ros2 humble이 설치되어있는 가정 하에 실행 가능합니다.
## 실행 순서
### 사용자용

### 개발자용
1. 본인 워크스페이스 내 src폴더의 git관련 파일들을 패키지 폴더 안에 넣기(.git, .gitignore 등)
2. src폴더 밑에서 git 저장소 생성
```bash
git init
```
3. git과 원격 저장소 연결
```bash
git remote add origin https://github.com/silsilkun/ROST_BOT.git
```
4. github의 main 폴더들 받기
```bash
git pull origin main
```
5. .gitignore에 개인 패키지 작성
6. 개발할 기능 브랜치로 이동
```bash
git switch <브랜치 이름>
```
7. 특정 브랜치 내에서는 특정 기능 패키지만 수정&Push

하준