import magnum as mn
import numpy as np

from habitat.articulated_agents.mobile_manipulator import (
    ArticulatedAgentCameraParams,
    MobileManipulator,
    MobileManipulatorParams,
)

class AlohaRobot(MobileManipulator):
    cls_uuid: str = "aloha_robot"

    def _get_base_params(self):
        # these refer to joints, seemingly sorted based on child anf joint name but quite unclear what order they are really in.
        left_front = [17, 18, 19, 20, 21, 22, 23, 24]
        right_front = [27, 28, 29, 30, 31, 32, 33, 34]
        left_rear = [37, 38, 39, 40, 41, 42, 43, 44]
        right_rear = [46, 47, 48, 49, 50, 51, 52, 53]
        arm_joints = left_front + right_front + left_rear + right_rear
        
        return MobileManipulatorParams(
            arm_joints=arm_joints,
            gripper_joints=[],
            wheel_joints=[10, 11],
            arm_init_params=np.zeros(len(arm_joints), dtype=np.float32),
            gripper_init_params=np.array([], dtype=np.float32),
            ee_offset=[mn.Vector3(0.08, 0, 0)],
            ee_links=[36],
            ee_constraint=np.array(
                [[[-0.08, 0.29], [-0.84, -0.27], [0.01, 1.12]]]
            ),
            cameras={
                "head": ArticulatedAgentCameraParams(
                    cam_offset_pos=mn.Vector3(0.0, 0.0, 0.0),
                    attached_link_id=3,
                    relative_transform=mn.Matrix4.rotation_y(mn.Deg(180)),
                    # @ mn.Matrix4.rotation_z(mn.Deg(-90)),
                ),
                "left_wrist": ArticulatedAgentCameraParams(
                    cam_offset_pos=mn.Vector3(0.05, 0.0, 0.0),
                    attached_link_id=25,
                    relative_transform=mn.Matrix4.rotation_y(mn.Deg(180)),
                    # @ mn.Matrix4.rotation_z(mn.Deg(-90)),
                ),
                "right_wrist": ArticulatedAgentCameraParams(
                    cam_offset_pos=mn.Vector3(0.05, 0.0, 0.0),
                    attached_link_id=35,
                    relative_transform=mn.Matrix4.rotation_y(mn.Deg(180)),
                    # @ mn.Matrix4.rotation_z(mn.Deg(-90)),
                ),
                "third": ArticulatedAgentCameraParams(
                    cam_offset_pos=mn.Vector3(0.8, 1, -0.5),
                    cam_look_at_pos=mn.Vector3(-1, 0.0, 0.75),
                    attached_link_id=-1,
                ),
            },
            gripper_closed_state=np.array([], dtype=np.float32),
            gripper_open_state=np.array([], dtype=np.float32),
            gripper_state_eps=0.01,
            arm_mtr_pos_gain=0.3,
            arm_mtr_vel_gain=0.3,
            arm_mtr_max_impulse=10.0,
            wheel_mtr_pos_gain=0.0,
            wheel_mtr_vel_gain=1.3,
            wheel_mtr_max_impulse=10.0,
            base_offset=mn.Vector3(0.0, 0.0, 0.0),
            base_link_names={
                "box1_Link",
                "base_link",
                "fl_base_link",
                "fr_base_link",
                "lr_base_link",
                "rr_base_link",
            },
        )

    @property
    def base_transformation(self):
        # TODO: this is possibly wrong
        add_rot = mn.Matrix4.rotation(
            mn.Rad(-np.pi / 2), mn.Vector3(1.0, 0, 0)
        )
        return self.sim_obj.transformation @ add_rot

    def __init__(
        self, urdf_path, sim, limit_robo_joints=True, fixed_base=True
    ):
        super().__init__(
            self._get_base_params(),
            urdf_path,
            sim,
            limit_robo_joints,
            fixed_base,
        )
