import rclpy
import DR_init
from rclpy.node import Node
from control.gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

# 속도 가속도 오프셋 (VEL, ACC 값만 수정하면됌)
VEL = 100
ACC = 50

BASE_VEL = 20.0
MAX_VEL = 100.0
WAIT_SEC_PER_VEL = 0.02

VEL = min(VEL, MAX_VEL)
wait_offset = max(0.0, VEL - BASE_VEL) * WAIT_SEC_PER_VEL
# 로봇팔 오프셋
PICK_APPROACH = 150
PICK_DESCENT = 120
LIFT= 250
PLACE_APPROACH = 150
PLACE_DESCENT = 20
# 그리퍼 오프셋
GRAB = 500
RELEASE = 0

# 분리수거 분류 ID
SORTING_ID = {"PAPER", "PLASTIC", "CAN"}

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class Recycle(Node):
    def __init__(self):
        super().__init__("recycle_node",namespace = ROBOT_ID)

        self.gripper = None
        self.gripper = GripperController(node=self, namespace = ROBOT_ID)

    # 입력 리스트를 검증하고 작업 딕셔너리로 전환
    def create_job(self, raw):
        if len(raw) != 7:
            raise ValueError(
                "데이터 형식은 [ID, pick_x, pick_y, pick_z, place_x, place_y, place_z] 이어야 한다"
            )
        item_id = str(raw[0]).upper()
        if item_id not in SORTING_ID:
            raise ValueError(f"unsupported ID: {item_id}")
        pick_xyz = (float(raw[1]), float(raw[2]), float(raw[3]))
        place_xyz = (float(raw[4]), float(raw[5]), float(raw[6]))
        return {"id": item_id, "pick": pick_xyz, "place": place_xyz}
    
    # 동작 시퀀스
    def pap_sequence(self, pick_xyz, place_xyz):
        from DSR_ROBOT2 import (
                movej,
                movel,
                ikin,
                posj,
                posx,
                wait,
                set_robot_mode,
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

        # pick 지점으로 하강
        pick_lower = posx(x1, y1, z1 + PICK_DESCENT, 90, 180, 90)
        movel(pick_lower, VEL, ACC)

        # pick 그리퍼 집기
        self.gripper.move(GRAB)
        wait(GRAB_WAIT)

        # pick 이후 상단 이동
        pick_up = posx(x1, y1, z1 + LIFT, 90, 180, 90)
        movel(pick_up, VEL, ACC)

        # place 지점 상단으로 이동
        place_upper = posx(x2, y2, z2 + PLACE_APPROACH, 90, 180, 90)
        movel(place_upper, VEL, ACC)

        # place 지점으로 하강
        place_lower = posx(x2, y2, z2 + PLACE_DESCENT, 90, 180, 90)
        movel(place_lower, VEL, ACC)

        # place 그리퍼 놓기
        self.gripper.move(RELEASE)
        wait(RELEASE_WAIT)

        # HOME 위치로 이동
        home = posj(0, 0, 90, 0, 90, 0)
        movej(home, VEL, ACC)

    # 데이터 > 작업 순차 처리
    def run(self, raw_jobs):
        for raw in raw_jobs:
            job = self.create_job(raw)
            self.pap_sequence(job["pick"], job["place"])
    
# 테스트용 데이터
def test_data():
    return [["PAPER", 500.0, 300, 130.0, 400.0, 100, 130.0],]

def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    test = Recycle()
    test.run(test_data())

    test.destroy_node()
    dsr_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
