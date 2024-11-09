import time
import numpy as np
import sapien.core as sapien
from sapien.utils.viewer import Viewer

def demo(fix_root_link=True, balance_passive_force=True):
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)

    scene_config = sapien.SceneConfig()
    scene = engine.create_scene(scene_config)
    scene.set_timestep(1 / 500.0)
    scene.add_ground(0)

    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    viewer = Viewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(x=-2, y=0, z=1)
    viewer.set_camera_rpy(r=0, p=-0.3, y=0)

    # Load URDF
    loader: sapien.URDFLoader = scene.create_urdf_loader()
    loader.fix_root_link = fix_root_link
    loader.load_multiple_collisions_from_file = True

    robot: sapien.Articulation = loader.load(
        "ManiSkill2_real2sim/mani_skill2_real2sim/assets/descriptions/g1_description/g1_29dof.urdf"
    )
    if robot is None:
        print("Failed to load the robot URDF.")
        return

    # Check active joints and DOF
    active_joints = robot.get_active_joints()
    print("Number of active joints:", len(active_joints))
    print("Active joints:", [joint.get_name() for joint in active_joints])
    print("Robot DOF:", robot.dof)

    if robot.dof == 0:
        print("Robot has no degrees of freedom. Please check the URDF and collision meshes.")
        return

    # Set initial pose
    robot.set_root_pose(sapien.Pose([0, 0, 0.5], [1, 0, 0, 0]))

    # Set initial joint positions
    qpos = [0.0] * robot.dof  # Adjust according to the number of joints
    robot.set_qpos(qpos)

    # Set joint drive properties
    for joint in robot.get_active_joints():
        joint.set_drive_property(stiffness=1e5, damping=1e3)

    # Remove the invalid method call
    # robot.set_drive_target(qpos)

    # Set drive targets for each joint individually
    for joint, target in zip(robot.get_active_joints(), qpos):
        joint.set_drive_target(target)

    while not viewer.closed:
        for _ in range(4):  # Render every 4 steps
            if balance_passive_force:
                qf = robot.compute_passive_force(
                    gravity=True,
                    coriolis_and_centrifugal=True,
                )
                robot.set_qf(qf)
            scene.step()
        scene.update_render()
        viewer.render()

def main():
    demo()

if __name__ == "__main__":
    main()
