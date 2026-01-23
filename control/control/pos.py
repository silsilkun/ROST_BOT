import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"


def _format_list(values):
    return [round(float(val), 1) for val in values]


def main(args=None):
    rclpy.init(args=args)
    dsr_node = rclpy.create_node("pos_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    try:
        from DSR_ROBOT2 import get_current_posx, get_current_posj

        posx, sol = get_current_posx()
        posj = get_current_posj()

        print("Current posx:", _format_list(posx))
        print("Solution space:", sol)
        print("Current posj:", _format_list(posj))
    finally:
        dsr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
