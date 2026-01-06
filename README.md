# ROST_BOT
본 프로젝트는 ROS2 Humble이 설치되어 있는 환경을 기준으로 동작합니다.
## 실행 순서
### 사용자용
추후 작성 예정
### 개발자용 (협업 세팅 가이드)
*이 프로젝트는 협업용 패키지들만 GitHub로 관리하며, 개인 참고용(대용량) 패키지는 로컬에만 유지합니다.*

1. 본인 워크스페이스 내 src폴더의 git관련 파일들을(있다면) 패키지 폴더 안에 넣기(.git, .gitignore 등)
```bash
ls -a # 숨겨진 파일 확인, 있다면 개인 패키지 안으로 옮기기
```

2. src폴더 밑에서 git 저장소 생성
```bash
git init # "src"라는 이름의 로컬 repo(저장소)가 생성됨
```

3. git과 원격 저장소 연결
```bash
git remote add origin https://github.com/silsilkun/ROST_BOT.git
```

4. github의 main 폴더들 받기
```bash
git fetch origin # origin(원격 저장소)의 커밋/브랜치 정보를 가져옴
git switch -c main origin/main # 로컬에 main 브랜치 생성 및 원격에 연결 -> 원격 저장소의 폴더를 가져옴, main으로 브랜치 변경
```
```bash
# 로컬 main이 필요한 이유
git switch main
git pull origin main # 로컬 main의 최신화
# 최신화를 위해 각각의 브랜치에서 upstream 설정 해주는게 좋음. -u 옵션
``` 

5. *.gitignore에 개인 패키지 작성(doosan-robot2와 같은 패키지)*

6. 작업할 기능 브랜치로 이동
```bash
git branch -a #브랜치 이름 확인
git switch <브랜치 이름> #브랜치 생성 및 이동 시 -c 옵션추가,  "<>"꺽쇠괄호는 제거
```
7. 특정 브랜치 내에서는 특정 기능 패키지만 수정&Push

하준
수환
승준
준표
진아

### 실행 방법
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch robot_system_bringup system.launch.py