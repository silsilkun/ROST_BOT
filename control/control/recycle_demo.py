import rclpy
from rclpy.node import Node
import DR_init
from gripper_drl_controller import GripperController

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

VEL = 20
ACC = 50

ZYZ_FIXED = (90.0, 180.0, 90.0)

APPROACH_OFFSET = 150.0
PICK_DESCEND = 30.0
LIFT_OFFSET = 100.0
PLACE_APPROACH_OFFSET = 150.0
PLACE_DESCEND = 10.0

VALID_IDS = {"PAPER", "PLASTIC", "CAN"}


class RecycleDemo(Node):
    def __init__(self):
        super().__init__("recycle_demo_node", namespace=ROBOT_ID)
        DR_init.__dsr__node = self
        from DSR_ROBOT2 import (
            movej,
            movel,
            posj,
            posx,
            wait,
            set_robot_mode,
            ROBOT_MODE_AUTONOMOUS,
        )

        self.movej = movej
        self.movel = movel
        self.posj = posj
        self.posx = posx
        self.wait = wait
        self.mode = set_robot_mode
        self.mode_auto = ROBOT_MODE_AUTONOMOUS
        self.gripper = GripperController(node=self, namespace=ROBOT_ID)
        self.home = self.posj(0, 0, 90, 0, 90, 0)
        self.close_pos = 600
        self.open_pos = 0
        self.close_wait = 2.5
        self.open_wait = 2.0

    def initialize(self):
        self.mode(self.mode_auto)
        self.movej(self.home, VEL, ACC)
        self.gripper.move(self.open_pos)

    def fixed_posx(self, x, y, z):
        a, b, c = ZYZ_FIXED
        return self.posx(x, y, z, a, b, c)

    def create_job(self, raw):
        if len(raw) != 7:
            raise ValueError(
                "job data must be [ID, pick_x, pick_y, pick_z, place_x, place_y, place_z]"
            )
        item_id = str(raw[0]).upper()
        if item_id not in VALID_IDS:
            raise ValueError(f"unsupported ID: {item_id}")
        pick_xyz = (float(raw[1]), float(raw[2]), float(raw[3]))
        place_xyz = (float(raw[4]), float(raw[5]), float(raw[6]))
        return {"id": item_id, "pick": pick_xyz, "place": place_xyz}

    def approach_pick(self, pick_xyz):
        x, y, z = pick_xyz
        target = self.fixed_posx(x, y, z + APPROACH_OFFSET)
        self.movel(target, VEL, ACC)

    def descend_pick(self, pick_xyz):
        x, y, z = pick_xyz
        target = self.fixed_posx(x, y, z + APPROACH_OFFSET - PICK_DESCEND)
        self.movel(target, VEL, ACC)

    def lift_after_pick(self, pick_xyz):
        x, y, z = pick_xyz
        target = self.fixed_posx(x, y, z + APPROACH_OFFSET + LIFT_OFFSET)
        self.movel(target, VEL, ACC)

    def approach_place(self, place_xyz):
        x, y, z = place_xyz
        target = self.fixed_posx(x, y, z + PLACE_APPROACH_OFFSET)
        self.movel(target, VEL, ACC)

    def descend_place(self, place_xyz):
        x, y, z = place_xyz
        target = self.fixed_posx(x, y, z + PLACE_APPROACH_OFFSET - PLACE_DESCEND)
        self.movel(target, VEL, ACC)

    def execute_job(self, job):
        pick_xyz = job["pick"]
        place_xyz = job["place"]
        self.approach_pick(pick_xyz)
        self.descend_pick(pick_xyz)
        self.gripper.move(self.close_pos)
        self.wait(self.close_wait)
        self.lift_after_pick(pick_xyz)
        self.approach_place(place_xyz)
        self.descend_place(place_xyz)
        self.gripper.move(self.open_pos)
        self.wait(self.open_wait)

    def run(self, raw_jobs):
        for raw in raw_jobs:
            job = self.create_job(raw)
            self.execute_job(job)


def get_job_data():
    return [
        ["PAPER", 400.0, -100.0, 50.0, 500.0, 0.0, 50.0],
    ]


def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    demo = RecycleDemo()
    demo.initialize()
    demo.run(get_job_data())
    demo.movej(demo.home, VEL, ACC)

    demo.gripper.terminate()
    demo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
