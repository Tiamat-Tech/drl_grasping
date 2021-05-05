#!/usr/bin/env python3

import time
from drl_grasping.control.panda_control import PandaControl


def main(args=None):

    panda_control = PandaControl(fci_ip="172.16.0.2",
                                 enforce_rt=False,
                                 dynamic_rel=0.005,
                                 gripper_speed=0.01,
                                 gripper_force=2,
                                 home_joint_positions=(0, 0, 0, -2.0, 0, 2.0, 0.79))

    panda_control.read()
    time.sleep(5)

    panda_control.gripper_open(width=50.0)
    time.sleep(10)

    panda_control.gripper_close()
    time.sleep(10)

    panda_control.move_home()
    time.sleep(30)

    panda_control.set_position_goal([0.5, 0.0, 0.25])
    panda_control.set_orientation_goal([0.0, 0.0, 0.0, 1.0],
                                       is_xyzw=True)
    panda_control.execute()
    time.sleep(10)


if __name__ == "__main__":
    main()
