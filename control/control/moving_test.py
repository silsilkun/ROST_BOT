# 시작 위치에서 목표 위치로 TCP가 목표 방향을 바라보도록 이동 테스트
import math
import sys

import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

VEL = 20
ACC = 10

JOINT_LIMITS_DEG = [
    (-360.0, 360.0),  # J1
    (-95.0, 95.0),    # J2
    (-135.0, 135.0),  # J3
    (-360.0, 360.0),  # J4
    (-135.0, 135.0),  # J5
    (-360.0, 360.0),  # J6
]


# ----- Input helpers -----

def _read_inputs():
    # CLI 인자 또는 입력으로 목표 XYZ 받기
    args = sys.argv[1:]
    if args:
        tokens = args
    else:
        raw = input("Enter target x y z (mm): ").strip()
        tokens = [tok for tok in raw.replace(",", " ").split() if tok]

    values = [float(tok) for tok in tokens]
    if len(values) != 3:
        raise ValueError(f"expected 3 values, got {len(values)}")
    return values[0], values[1], values[2]


def _unit(vec):
    # 벡터 정규화
    norm = math.sqrt(sum(v * v for v in vec))
    if norm == 0:
        raise ValueError("direction vector is zero")
    return [v / norm for v in vec]


def _dot(a, b):
    # x축을 만들기 위한 보조 계산
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    # 축을 만드는 도구
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _look_at_zyz(cur_xyz, target_xyz):
    # 현재 TCP 위치에서 목표 위치로 향하는 방향 벡터
    direction = [
        target_xyz[0] - cur_xyz[0],
        target_xyz[1] - cur_xyz[1],
        target_xyz[2] - cur_xyz[2],
    ]
    # 목표 방향 벡터를 단위벡터로 만들기 
    z_axis = _unit(direction)
    # 기준이 되는 베이스 좌표계의 x+ 방향
    base_x = [1.0, 0.0, 0.0]
    # 현재 base_x랑 z_axis 방향이 얼마나 곂치는지 계산 후 x축 후보 만들기
    proj = _dot(base_x, z_axis)
    x_axis = [base_x[i] - proj * z_axis[i] for i in range(3)]
    # x축 후보가 0이 되는 특수 상황 예외 처리
    try:
        x_axis = _unit(x_axis)
    except ValueError:
        fallback = [0.0, 1.0, 0.0]
        proj = _dot(fallback, z_axis)
        x_axis = _unit([fallback[i] - proj * z_axis[i] for i in range(3)])
    # z축을 기준으로 x축과 y축을 직교하게 재정렬
    y_axis = _cross(z_axis, x_axis)
    x_axis = _cross(y_axis, z_axis)
    # TCP 좌표계의 축이 베이스 좌표 기준 어떻게 보이는지 나타내는 회전행렬
    rotm = [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]
    return _rotm_to_zyz(rotm)


def _rotm_to_zyz(rotm):
    # ZYZ 오일러각 변환 공식으로 회전행렬 -> ZYZ 변환
    r11, r12, r13 = rotm[0]
    r21, r22, r23 = rotm[1]
    r31, r32, r33 = rotm[2]

    b = math.acos(max(-1.0, min(1.0, r33)))
    sin_b = math.sin(b)
    if abs(sin_b) < 1e-8:
        a = math.atan2(r21, r11)
        c = 0.0
    else:
        a = math.atan2(r23, r13)
        c = math.atan2(r32, -r31)

    return (math.degrees(a), math.degrees(b), math.degrees(c))

def run_test(target_xyz):
    from DSR_ROBOT2 import (
        movej,
        movel,
        ikin,
        posx,
        get_current_posx,
        get_current_posj,
        set_robot_mode,
        DR_BASE,
        ROBOT_MODE_AUTONOMOUS,
    )
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 현재 TCP/관절 위치 읽기
    cur_posx, _ = get_current_posx()
    cur_posj = get_current_posj()
    if not cur_posx or len(cur_posx) < 6:
        raise RuntimeError("get_current_posx returned invalid data")

    def _to_list(q):
        # 리스트 형식으로 정규화
        return q.tolist() if hasattr(q, "tolist") else list(q)

    def _within_limits(q_list):
        # 제한 관절값 반영 Joint_Limit
        for idx, val in enumerate(q_list):
            mn, mx = JOINT_LIMITS_DEG[idx]
            if val < mn or val > mx:
                return False
        return True

    def _sum_delta(q_list, cur_list):
        # 현재 관절과의 변화량(해 선택 기준)
        return sum(abs(q_list[i] - cur_list[i]) for i in range(6))

    cur_list = _to_list(cur_posj)

    # 목표 방향으로 50mm 떨어진 접근 지점
    approach_offset = 50.0  

    best = None  # (cost, sol, q_approach, q_target, a, b, c)
    # 현재->목표 방향을 바라보는 TCP 자세(ZYZ) 계산
    a, b, c = _look_at_zyz(cur_posx[:3], target_xyz)
    direction = [
        target_xyz[0] - cur_posx[0],
        target_xyz[1] - cur_posx[1],
        target_xyz[2] - cur_posx[2],
    ]
    direction_unit = _unit(direction)
    # 접근 지점: 목표점에서 진행 방향 반대쪽으로 50mm 이동
    approach_xyz = [
        target_xyz[0] - direction_unit[0] * approach_offset,
        target_xyz[1] - direction_unit[1] * approach_offset,
        target_xyz[2] - direction_unit[2] * approach_offset,
    ]

    # 목표/접근 포즈 생성(자세는 동일)
    target_pose = posx(target_xyz[0], target_xyz[1], target_xyz[2], a, b, c)
    approach_pose = posx(
        approach_xyz[0],
        approach_xyz[1],
        approach_xyz[2],
        a,
        b,
        c,
    )
    # sol 0~7 중 관절 제한 만족 + 현재 관절 변화가 최소인 해 선택
    for sol in range(8):
        try:
            q_target = ikin(target_pose, sol, DR_BASE)
            q_approach = ikin(approach_pose, sol, DR_BASE)
        except TypeError:
            q_target = ikin(target_pose, sol)
            q_approach = ikin(approach_pose, sol)
        if q_target is None or q_approach is None:
            continue
        q_target_list = _to_list(q_target)
        q_approach_list = _to_list(q_approach)
        if not _within_limits(q_target_list):
            continue
        if not _within_limits(q_approach_list):
            continue
        cost = _sum_delta(q_target_list, cur_list)
        if best is None or cost < best[0]:
            best = (cost, sol, q_approach_list, q_target_list, a, b, c)

    # 현재 상태 출력
    print("Current posx:", [round(v, 1) for v in cur_posx])
    print("Current posj:", [round(v, 1) for v in cur_list])
    if best is None:
        print("No valid IK solution within joint limits for TCP-down poses.")
        return

    _, sol, q_approach_list, q_target_list, a, b, c = best
    print(f"Selected ZYZ: A={a:.2f}, B={b:.2f}, C={c:.2f}, sol: {sol}")
    # J6 고정(그리퍼 회전 고정)
    q_approach_list[5] = 0.0
    q_target_list[5] = 0.0
    # 접근 지점까지 movej
    movej(q_approach_list, v=VEL, a=ACC)
    # 접근 후 실제 자세를 유지하기 위해 목표좌표 재설정
    cur_after_posx, _ = get_current_posx()
    if not cur_after_posx or len(cur_after_posx) < 6:
        raise RuntimeError("get_current_posx returned invalid data after approach movej")
    target_pose_fixed = posx(
        target_xyz[0],
        target_xyz[1],
        target_xyz[2],
        cur_after_posx[3],
        cur_after_posx[4],
        cur_after_posx[5],
    )
    # 목표점으로 직선 이동
    movel(target_pose_fixed, vel=VEL, acc=ACC)


def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("moving_test_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    try:
        x, y, z = _read_inputs()
        run_test([x, y, z])
    finally:
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
