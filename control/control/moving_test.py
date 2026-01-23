# 보통 6축 산업용 로봇은 이론적으로 최대 8개의 해가 나옵니다.
# 어깨 좌/우 × 팔꿈치 상/하 × 손목 플립/노플립 = 2×2×2
# 목표 자세가 특이점에 가깝거나 도달 불가 위치면 해의 값이 0 ~ 몇개만 나올수도 있음

# 어깨 좌/우 는 즉 왼팔을 뻗는것처럼 할꺼냐 아니면 오른팔을 뻗는것처럼 할꺼냐
# 주로 해의 값에 0~3 : Lefty , 4~7 : Righty

# 팔꿈치 상/하 -> 팔꿈치가 기준선보다 위로 꺾였는지, 아래로 꺾였는지 주로 J2 + J3 조합
# 두산 기준 sol 매핑 0,1-> Lefty Below  2,3-> Lefty Above  4,5->Righty Below 6,7->  Righty Above

# 손목 플립은 주로 6번축 (또는 4~6번축 손목부)의 방향
# 핵심은 6번축이지만 손목 4~6축 전체의 배치 변화

import sys
import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

VEL = 20
ACC = 10


def _parse_floats(tokens, expected_counts):
    values = [float(tok) for tok in tokens]
    if len(values) not in expected_counts:
        raise ValueError(f"expected {expected_counts} values, got {len(values)}")
    return values


# 솔루션 출력을 (관절값 / status 값) 으로 나타냄
# status -> 0: 조인트 값 정상 반환  1: 동작 영역을 벗어남 2: 손목 축 특이점 
def list_ikin_solutions(target_posx, ikin_fn, ref):
    solutions = {}
    for sol_space in range(8):
        try:
            try:
                result = ikin_fn(target_posx, sol_space, ref, ref_pos_opt=0)
            except TypeError:
                try:
                    result = ikin_fn(target_posx, sol_space, ref, 0)
                except TypeError:
                    try:
                        result = ikin_fn(target_posx, sol_space, ref)
                    except TypeError:
                        result = ikin_fn(target_posx, sol_space)
            if isinstance(result, (list, tuple)) and len(result) == 2:
                q, status = result
            else:
                q, status = result, None
            solutions[sol_space] = (q, status)
        except Exception as exc:
            solutions[sol_space] = (None, f"error: {exc}")
    return solutions

# 터미널 출력값 재정의 소수점 한자리 반올림
def _format_angles(q):
    q_list = q.tolist() if hasattr(q, "tolist") else list(q)
    return [round(float(val), 1) for val in q_list]


def run_test(x, y, z, a, b, c):
    from DSR_ROBOT2 import (
        movej,
        ikin,
        posx,
        set_robot_mode,
        DR_BASE,
        ROBOT_MODE_AUTONOMOUS,
    )

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 입력받은 posx 값에 대한 solution값들을 형식에 맞게 터미널에 출력
    target = posx(x, y, z, a, b, c)
    solutions = list_ikin_solutions(target, ikin, DR_BASE)

    print("\nTarget posx:", [x, y, z, a, b, c])
    print("Solution space -> joint angles (deg) / status")
    for sol_space in range(8):
        q, status = solutions[sol_space]
        if q is None:
            print(f"  {sol_space}: None / {status}")
        else:
            print(f"  {sol_space}: {_format_angles(q)} / {status}")

    # 출력된 solution값들 중 어떻게 진행할껀지 입력받고 진행
    while True:
        choice = input("\nMove with solution space (0-7) or q to quit: ").strip().lower()
        if choice in ("q", "quit", "exit"):
            return
        if not choice.isdigit():
            print("Please enter a number 0-7 or q.")
            continue
        sol_space = int(choice)
        if sol_space not in solutions:
            print("Invalid solution space.")
            continue
        q, status = solutions[sol_space]
        if q is None:
            print(f"Solution {sol_space} is not available: {status}")
            continue
        print(f"Moving with solution {sol_space} (status: {status})...")
        q_list = q.tolist() if hasattr(q, "tolist") else q
        movej(q_list, v=VEL, a=ACC)

# 터미널에 입력한 값을 쉼표/공백 분리후 읽고 반환
def _read_inputs():
    args = sys.argv[1:]
    if args:
        values = _parse_floats(args, expected_counts=(6,))
        return values[0], values[1], values[2], values[3], values[4], values[5]

    raw = input("Enter x y z a b c (mm, mm, mm, deg, deg, deg): ").strip()
    tokens = [tok for tok in raw.replace(",", " ").split() if tok]
    values = _parse_floats(tokens, expected_counts=(6,))
    return values[0], values[1], values[2], values[3], values[4], values[5]


def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("moving_test_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    try:
        x, y, z, a, b, c = _read_inputs()
        run_test(x, y, z, a, b, c)
    finally:
        dsr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
