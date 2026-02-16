<<<<<<< HEAD
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




### 실행 방법
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch robot_system_bringup system.launch.py

numpy 버전: 1.25.2
scipy 버전: 1.15.3
# 260113 Report
1) 기존에 /utils에 있던 judge_node.py를 /nodes로 이동
2) sample test 성공 -> 모든 객체 정확하게 분류
3) plastic의 경우, '파란색 boxing된 객체는 플라스틱'이라는 강력한 프롬프트로 분류
4) test 진행 시, 0113_test.zip 파일을 src 폴더랑 같은 위치에 압축 해제하기 (test_one_shot_pub.py + test_images/sample_01.jpg)
# 260127 Report
1) estimation_node로 통합
2) estimation 새로 짜기 시작

# 260129 Report
1) test_client 성공
2) estimation 단독 실행으로 성공
3) rost_interfaces 수정
=======
# 260210 Tuesday
- 전체 쓰레기 더미 분류 완료 
- 오분류 2개 (토마토 캔, 찌그러진 플라스틱 컵 = vinyl)
- 벽면에 붙은 쓰레기 처리 시퀀스 내일 구상

# 260211 Wednesday
- 프롬프트 수정 => 자연어 기반 구어체 프롬프트 (추론 유도)
- 오분류 1개 (찌그러진 플라스틱 컵 = vinyl)
 - 찌그러진 플라스틱 컵은 배제하기
- 겹쳐진 물건은 배경에 있는 물체를 인식함 (e.g. 박스 위에 토마토캔 위치 시 box로 분류)
- 환경을 쫓아가지 말고, demo에서 가장 잘 보여줄 수 있는 환경으로 구성하기
 - "unknown" 분류 보여주기 위한 이물질이 담긴 플라스틱 병 준비
 - 토마토캔은 항상 뚜껑을 밖으로 빼두기 (손 조심)
 - 물체들끼리 최대한 겹쳐지지 않게 하기 (살짝 걸쳐있는 건 문제가 안되지만 아예 포개져있는 물체는 오류 발생)
- gemini_functions_v2.py가 0211 최종 ver
- main_pipeline.py은 수환님 시퀀스 파악용 코드
- test_pipeline.py 실행 시 노드 없이 전체 utils 테스트 가능
>>>>>>> 0210-per+est
