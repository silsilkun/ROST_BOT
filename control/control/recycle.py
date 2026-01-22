import rclpy
import DR_init
from rclpy.node import Node
from control.gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

# 속도 가속도 오프셋 (VEL, ACC 값만 수정하면됌)
VEL = 100
ACC = 50

# wait 오프셋
BASE_VEL = 20.0
MAX_VEL = 100.0
WAIT_SEC_PER_VEL = 0.03  # 이 값을 수정

VEL = min(VEL, MAX_VEL)
wait_offset = max(0.0, VEL - BASE_VEL) * WAIT_SEC_PER_VEL

# 로봇팔 오프셋
PICK_APPROACH = 150
PICK_DESCENT = 90
LIFT = 280
PLACE_APPROACH = 300
PLACE_DESCENT = 130
LIFT_2 = 250

# 그리퍼 오프셋
GRAB = 500
RELEASE = 0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class Recycle(Node):
    def __init__(self):
        super().__init__("recycle_node",namespace = ROBOT_ID)

        self.gripper = None
        self.gripper = GripperController(node=self, namespace = ROBOT_ID)

    # 로봇팔의 현재 포즈
    def get_posx(self, get_current_posx, wait_fn, retries=3, delay=0.1):
        for attempt in range(1, retries + 1):
            try:
                result = get_current_posx()
            except Exception as exc:
                self.get_logger().warn(f"get_current_posx failed (attempt {attempt}/{retries}): {exc}")
                result = None

            if result:
                cur_posx, sol = result
                if cur_posx is not None:
                    try:
                        if len(cur_posx) >= 6:
                            return cur_posx, sol
                    except TypeError:
                        pass

            if wait_fn:
                wait_fn(delay)

        return None, None

    # trash_list -> 5개씩 묶인 2차원 리스트로 재정의
    def normalize_trash_list(self, trash_list):
        if not trash_list:
            return []
        if isinstance(trash_list[0], (list, tuple)):
            return trash_list
        return [
            trash_list[i:i + 5] for i in range(0, len(trash_list), 5)
            if len(trash_list[i:i + 5]) == 5
        ]

    # 입력 리스트를 검증하고 작업 딕셔너리로 전환
    def create_job(self, trash, bin):
        item_id = self.type_id(trash[0]) # type이 플라스틱일때 높이값 고정 그 외에는 그대로
        if float(trash[0]) == 0.0:
            z = 160.0
        else:
            z = float(trash[3]) * 10.0

        # 데이터값 로봇팔 기준으로 변경
        pick_xyz = (float(trash[1]) * 10.0, float(trash[2]) * 10.0, z)
        angle = float(trash[4])

        # type이 PLASTIC일 경우 그리퍼값 고정
        grab_offset = 220 if float(trash[0]) == 0.0 else 0
        place_xyz = (float(bin[0]) * 10.0, float(bin[1]) * 10.0, 140)
        return {"id": item_id, "pick": pick_xyz, "angle": angle, "place": place_xyz, "grab_offset": grab_offset}
    
    # 동작 시퀀스
    def pap_sequence(self, pick_xyz, grip_angle, place_xyz, grab_offset=0):
        from DSR_ROBOT2 import (
                movej,
                movel,
                ikin,
                posj,
                posx,
                wait,
                set_robot_mode,
                get_current_posj,
                get_current_posx,
                DR_BASE,
                ROBOT_MODE_AUTONOMOUS,
            )

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        x1, y1, z1 = pick_xyz
        x2, y2, z2 = place_xyz
        GRAB_WAIT = 2.0 + wait_offset
        RELEASE_WAIT = 2.5 + wait_offset

        # HOME 위치 초기화
        home = posj(0, 0, 90, 0, 90, 0)
        movej(home, VEL, ACC)
        self.gripper.move(RELEASE)
        wait(RELEASE_WAIT)

        # pick 지점 상단으로 이동
        pick_upper = posx(x1, y1, z1 + PICK_APPROACH, 90, 180, 90)
        movel(pick_upper, VEL, ACC)

        # 물체 각도 만큼 회전
        q = get_current_posj()
        gripper_turn = posj(q[0], q[1], q[2], q[3], q[4], q[5] + grip_angle)
        movej(gripper_turn,VEL, ACC)

        # 현재 자세의 회전값을 유지 ( 여기선 그리퍼가 수직으로 아래를 유지하기 위함 )
        cur_posx, _ = self.get_posx(get_current_posx, wait)
        if cur_posx is None:
            self.get_logger().error("get_current_posx returned empty data; aborting sequence")
            return
        rx, ry, rz = cur_posx[3], cur_posx[4], cur_posx[5]

        # pick 지점으로 하강
        pick_lower = posx(x1, y1, z1 + PICK_DESCENT, rx, ry, rz)
        movel(pick_lower, VEL, ACC)

        # pick 그리퍼 집기
        self.gripper.move(GRAB + grab_offset)
        wait(GRAB_WAIT)

        # pick 이후 상단 이동
        pick_up = posx(x1, y1, z1 + LIFT, rx, ry, rz)
        movel(pick_up, VEL, ACC)

        # place 지점 상단으로 이동
        place_upper = posx(x2, y2, z2 + PLACE_APPROACH, rx, ry, rz)
        movel(place_upper, VEL, ACC)

        # place 지점으로 하강
        place_lower = posx(x2, y2, z2 + PLACE_DESCENT, rx, ry, rz)
        movel(place_lower, VEL, ACC)

        # place 그리퍼 놓기
        self.gripper.move(RELEASE)
        wait(RELEASE_WAIT)

        # place 이후 상단 이동
        place_lift = posx(x2, y2, z2 + LIFT_2, rx, ry, rz)
        movel(place_lift, VEL, ACC)

        # HOME 위치로 이동
        home = posj(0, 0, 90, 0, 90, 0)
        movej(home, VEL, ACC)

    # 로봇팔 작업 순서
    def run(self, trash_list, bin_list):

        # 받아오는 데이터 trash_list , bin_list 좌표값 디버깅
        debug_trash = []
        for item in trash_list:
            if not item:
                continue
            scaled = [item[0]]
            for idx, value in enumerate(item[1:], start=1):
                if idx == 4:
                    scaled.append(round(float(value) , 2))
                else:
                    scaled.append(round(float(value) * 10.0, 2))
            debug_trash.append(scaled)

        debug_bin = []
        for item in bin_list:
            if not item:
                continue
            debug_bin.append([round(float(v) * 10.0, 2) for v in item])

        print("[Recycle]")
        print("trash_list=[")
        for item in debug_trash:
            print(f"{item},")
        print("]")

        print("[Recycle]")
        print("bin_list=[")
        for item in debug_bin:
            print(f"{item},")
        print("]")

        # 처리할 정보가 없으면 작업 중단
        trash_items = self.normalize_trash_list(trash_list)
        if not trash_items or not bin_list:
            return

        # type별로 bin을 하나씩 매칭 같을 시 같은 bin으로 계속 처리
        type_to_bin = {}
        next_bin_index = 0
        for trash in trash_items:
            item_type = self.type_id(trash[0])

            # 타입이 처음 나오면 다음 bin 할당, bin이 부족하면 처리안함
            if item_type not in type_to_bin:
                if next_bin_index >= len(bin_list):
                    continue
                type_to_bin[item_type] = bin_list[next_bin_index]
                next_bin_index += 1

            # 해당 타입이 매핑될 bin이 없으면 처리하지 않음
            bin_data = type_to_bin.get(item_type)
            if not bin_data:
                continue

            job = self.create_job(trash, bin_data)
            self.pap_sequence(job["pick"], job["angle"], job["place"], job["grab_offset"])
            print(f"{item_type}을 분리수거 완료했습니다")

    # ID별로 type 이름 지정해주기
    def type_id(self, value):
        mapping = {
            0.0: "PLASTIC",
            1.0: "CAN",
            2.0: "PAPER",
            3.0: "BOX",
            -1.0: "UNKNOWN",
        }
        try:
            return mapping.get(float(value), str(value).upper())
        except (TypeError, ValueError):
            return str(value).upper()
    
# 테스트용 데이터
def test_data():
    trash = [["PAPER", 400, 200, 145, 50],]
    bin = [[300,430],]
    return trash, bin

def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    test = Recycle()
    trash, bin = test_data()
    test.run(trash, bin)

    test.destroy_node()
    dsr_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
