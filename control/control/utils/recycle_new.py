import rclpy
import DR_init
from rclpy.node import Node
from control.control.utils.gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

# 속도 가속도 오프셋 (VEL, ACC 값만 수정하면됨)
VEL = 50
ACC = 30

# wait 오프셋
BASE_VEL = 20.0
MAX_VEL = 100.0
WAIT_SEC_PER_VEL = 0.03

VEL = min(VEL, MAX_VEL)
wait_offset = max(0.0, VEL - BASE_VEL) * WAIT_SEC_PER_VEL

# 로봇팔 오프셋
PICK_APPROACH = 150
PICK_DESCENT = 90
LIFT = 280

# 그리퍼 오프셋
GRAB = 500
RELEASE = 0

# 관절 제한 (deg)
JOINT_LIMITS_DEG = [
    (-360.0, 360.0),  # J1
    (-95.0, 95.0),    # J2
    (-135.0, 135.0),  # J3
    (-360.0, 360.0),  # J4
    (-135.0, 135.0),  # J5
    (-360.0, 360.0),  # J6
]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class RecycleNew(Node):
    def __init__(self):
        super().__init__("recycle_new_node", namespace=ROBOT_ID)
        self.gripper = GripperController(node=self, namespace=ROBOT_ID)

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
    def create_job(self, trash, bin_pos):
        item_id = self.type_id(trash[0])
        if float(trash[0]) == 0.0:
            z = 160.0
        else:
            z = float(trash[3])

        pick_xyz = (float(trash[1]), float(trash[2]), z)
        angle = float(trash[4])

        grab_offset = 220 if float(trash[0]) == 0.0 else 0
        place_xyz = (float(bin_pos[0]), float(bin_pos[1]), 250)
        return {"id": item_id, "pick": pick_xyz, "angle": angle, "place": place_xyz, "grab_offset": grab_offset}

    # ---------- moving_test helpers ----------
    def _unit(self, vec):
        norm = sum(v * v for v in vec) ** 0.5
        if norm == 0:
            raise ValueError("direction vector is zero")
        return [v / norm for v in vec]

    def _dot(self, a, b):
        return sum(x * y for x, y in zip(a, b))

    def _cross(self, a, b):
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]

    def _rotm_to_zyz(self, rotm):
        r11, r12, r13 = rotm[0]
        r21, r22, r23 = rotm[1]
        r31, r32, r33 = rotm[2]

        import math
        b = math.acos(max(-1.0, min(1.0, r33)))
        sin_b = math.sin(b)
        if abs(sin_b) < 1e-8:
            a = math.atan2(r21, r11)
            c = 0.0
        else:
            a = math.atan2(r23, r13)
            c = math.atan2(r32, -r31)

        return (math.degrees(a), math.degrees(b), math.degrees(c))

    def _look_at_zyz(self, cur_xyz, target_xyz):
        direction = [
            target_xyz[0] - cur_xyz[0],
            target_xyz[1] - cur_xyz[1],
            target_xyz[2] - cur_xyz[2],
        ]
        z_axis = self._unit(direction)
        base_x = [1.0, 0.0, 0.0]
        proj = self._dot(base_x, z_axis)
        x_axis = [base_x[i] - proj * z_axis[i] for i in range(3)]
        try:
            x_axis = self._unit(x_axis)
        except ValueError:
            fallback = [0.0, 1.0, 0.0]
            proj = self._dot(fallback, z_axis)
            x_axis = self._unit([fallback[i] - proj * z_axis[i] for i in range(3)])
        y_axis = self._cross(z_axis, x_axis)
        x_axis = self._cross(y_axis, z_axis)
        rotm = [
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]],
        ]
        return self._rotm_to_zyz(rotm)

    def _select_ik_solution(self, ikin, target_pose, cur_posj, DR_BASE, j4_sign=None):
        def _to_list(q):
            return q.tolist() if hasattr(q, "tolist") else list(q)

        def _within_limits(q_list):
            for idx, val in enumerate(q_list):
                mn, mx = JOINT_LIMITS_DEG[idx]
                if val < mn or val > mx:
                    return False
            return True

        def _sum_delta(q_list, cur_list):
            return sum(abs(q_list[i] - cur_list[i]) for i in range(6))

        cur_list = _to_list(cur_posj)

        best = None
        best_j4_delta = None
        for sol in range(8):
            try:
                q_target = ikin(target_pose, sol, DR_BASE)
            except TypeError:
                q_target = ikin(target_pose, sol)
            if q_target is None:
                continue
            q_target_list = _to_list(q_target)
            if j4_sign == "positive" and q_target_list[3] <= 0.0:
                continue
            if j4_sign == "negative" and q_target_list[3] >= 0.0:
                continue
            if not _within_limits(q_target_list):
                continue
            cost = _sum_delta(q_target_list, cur_list)
            j4_delta = abs(q_target_list[3] - cur_list[3])
            if best is None or cost < best[0] or (cost == best[0] and j4_delta < best_j4_delta):
                best = (cost, sol, q_target_list)
                best_j4_delta = j4_delta
        return best

    # ---------- main sequence ----------
    def run_job(self, pick_xyz, grip_angle, place_xyz, grab_offset=0):
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
        RELEASE_WAIT = 2.0 + wait_offset

        # HOME 위치 초기화
        home = posj(0, 0, 90, 0, 90, 0)
        movej(home, VEL, ACC)
        self.gripper.move(RELEASE)
        wait(RELEASE_WAIT)

        # pick 지점 상단으로 이동 (movej)
        cur_posx, _ = self.get_posx(get_current_posx, wait)
        if cur_posx is None:
            self.get_logger().error("get_current_posx returned empty data; aborting sequence")
            return
        rx_home, ry_home, rz_home = cur_posx[3], cur_posx[4], cur_posx[5]
        pick_upper = posx(x1, y1, z1 + PICK_APPROACH, rx_home, ry_home, rz_home)
        cur_posj = get_current_posj()
        best = self._select_ik_solution(ikin, pick_upper, cur_posj, DR_BASE)
        if best is None:
            self.get_logger().error("No valid IK solution within joint limits for pick upper")
            return
        _, sol, q_target_list = best
        self.get_logger().info(f"Selected IK sol={sol} for pick upper")
        q_target_list[5] = grip_angle
        movej(q_target_list, v=VEL, a=ACC)

        # 현재 자세의 회전값 유지
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
        cur_posj = get_current_posj()
        best = self._select_ik_solution(ikin, pick_up, cur_posj, DR_BASE)
        if best is None:
            self.get_logger().error("No valid IK solution within joint limits for pick up")
            return
        _, sol, q_target_list = best
        self.get_logger().info(f"Selected IK sol={sol} for pick up")
        q_target_list[5] = 0.0
        movej(q_target_list, v=VEL, a=ACC,r=200)

        # pick 이후 HOME 위치로 이동
        home = posj(0, 0, 90, 0, 90, 0)
        movej(home, VEL, ACC,r=50)

        # 기준점에서 place 방향으로 접근 포즈 계산
        cur_posj = get_current_posj()
        ref_xyz = [370.0, 0.0, 500.0]
        a, b, c = self._look_at_zyz(ref_xyz, [x2, y2, z2])
        direction = [
            x2 - ref_xyz[0],
            y2 - ref_xyz[1],
            z2 - ref_xyz[2],
        ]
        direction_unit = self._unit(direction)
        target_pose = posx(x2, y2, z2, a, b, c)
        j4_sign = "positive" if y2 >= 0.0 else "negative"
        best = self._select_ik_solution(ikin, target_pose, cur_posj, DR_BASE, j4_sign=j4_sign)
        if best is None:
            self.get_logger().error("No valid IK solution within joint limits for place approach")
            return

        _, sol, q_target_list = best
        self.get_logger().info(f"Selected IK sol={sol} for place approach")

        q_target_list[5] = 0.0

        # 목표점으로 movej 이동
        movej(q_target_list, v=VEL, a=ACC)

        # movej 후 현재 자세 가져오기 (하강용 자세)
        cur_after_posx, _ = get_current_posx()
        if not cur_after_posx or len(cur_after_posx) < 6:
            raise RuntimeError("get_current_posx returned invalid data after target movej")

        self.gripper.move(RELEASE)
        wait(RELEASE_WAIT)

        # HOME 위치로 이동
        home = posj(0, 0, 90, 0, 90, 0)
        movej(home, VEL, ACC)

    # 분리수거 처리 순서
    def run(self, trash_list, bin_list):
        trash_items = self.normalize_trash_list(trash_list)
        if not trash_items or not bin_list:
            return

        type_to_bin = {}
        next_bin_index = 0
        for trash in trash_items:
            item_type = self.type_id(trash[0])

            if item_type not in type_to_bin:
                if next_bin_index >= len(bin_list):
                    continue
                type_to_bin[item_type] = bin_list[next_bin_index]
                next_bin_index += 1

            bin_data = type_to_bin.get(item_type)
            if not bin_data:
                continue

            job = self.create_job(trash, bin_data)
            self.run_job(job["pick"], job["angle"], job["place"], job["grab_offset"])
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
    trash = [[2.0, 400, 200, 145, 50]]
    bin_pos = [[600, -300]]
    return trash, bin_pos


def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("dsr_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node

    test = RecycleNew()
    trash, bin_pos = test_data()
    test.run(trash, bin_pos)

    test.destroy_node()
    dsr_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
