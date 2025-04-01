#!/usr/bin/env python
"""
| File: 8_camera_vehicle.py
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to build an app that makes use of the Pegasus API,
| where the data is send/received through mavlink, the vehicle is controled using mavlink and
| camera data is sent to ROS2 topics at the same time.
"""

# Import argparse for command-line argument parsing
import argparse
import numpy as np

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp


# Parse command-line arguments
# Note: We need to parse arguments before initializing SimulationApp
def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Pegasus API Example with configurable PX4 directory."
    )
    parser.add_argument(
        "--px4_dir", type=str, required=True, help="Path to the PX4-Autopilot directory"
    )
    return parser.parse_args()


# Parse arguments
args = parse_arguments()

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation


class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self, px4_dir):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.

        Args:
            px4_dir (str): Path to the PX4-Autopilot directory
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        # self.pg.load_environment(SIMULATION_ENVIRONMENTS["Black Gridroom"])
        self.pg.load_environment(
            "/mnt/a/alejodosr/omniverse-usd/SHEREC - OmniverseUSD/ship_interior.usd"
        )

        # from omni.isaac.core.objects import DynamicCuboid
        # import numpy as np
        # cube_2 = self.world.scene.add(
        #     DynamicCuboid(
        #         prim_path="/new_cube_2",
        #         name="cube_1",
        #         position=np.array([-3.0, 0, 2.0]),
        #         scale=np.array([1.0, 1.0, 1.0]),
        #         size=1.0,
        #         color=np.array([255, 0, 0]),
        #     )
        # )

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = MultirotorConfig()
        # Create the multirotor configuration
        mavlink_config = PX4MavlinkBackendConfig(
            {
                "vehicle_id": 0,
                "px4_autolaunch": True,
                "px4_dir": px4_dir,  # Use the provided PX4 directory path
            }
        )
        config_multirotor.backends = [
            PX4MavlinkBackend(mavlink_config),
            ROS2Backend(
                vehicle_id=1,
                config={
                    "namespace": "drone",
                    "pub_sensors": True,
                    "pub_graphical_sensors": True,
                    "pub_state": False,
                    "sub_control": False,
                },
            ),
        ]

        # Create a camera and lidar sensors
        config_multirotor.graphical_sensors = [
            MonocularCamera(
                "camera",
                config={
                    "update_rate": 60.0,
                    "intrinsics": np.array(
                        [[609.25, 0.0, 960.0], [0.0, 609.25, 540.0], [0.0, 0.0, 1.0]]
                    ),
                    "distortion_coefficients": np.array(
                        [
                            5.618029247726649800,
                            2.633097129851353024,
                            -1.839432678176205083e-05,
                            -4.505550891889315750e-04,
                            -1.634711859741078044e01,
                            5.107694839223768213e00,
                            3.699287666569800681e00,
                            -1.668676891240622950e01,
                            0.000000000000000000e00,
                            0.000000000000000000e00,
                            0.000000000000000000e00,
                            0.000000000000000000e00,
                            0.000000000000000000e00,
                            0.000000000000000000e00,
                        ]
                    ),
                    "diagonal_fov": 127.0,
                },
            ),
            Lidar("lidar"),
        ]  # Lidar("lidar")

        Multirotor(
            "/World/quadrotor",
            ROBOTS["Iris"],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

        # Add this at the end of your __init__ method
        self.setup_simple_camera()

    # Add this to your PegasusApp class after setting up the world but before the main loop


    def setup_simple_camera(self):
        """
        Create a camera that will follow the quadrotor and look at it
        """
        from pxr import UsdGeom, Gf, Sdf
        import math

        # Create a camera prim
        camera_path = "/World/SimpleFollowCamera"
        camera = UsdGeom.Camera.Define(self.world.stage, camera_path)

        # Set the focal length to 5.0mm (adjust horizontal and vertical aperture)
        # Standard 35mm camera has 36mm × 24mm aperture
        # Reducing focal length while keeping aperture the same gives wider FOV
        camera.GetFocalLengthAttr().Set(30.0)
        camera.GetHorizontalApertureAttr().Set(36.0)  # Standard 35mm film width
        camera.GetVerticalApertureAttr().Set(24.0)  # Standard 35mm film height

        # Store references
        self.camera_path = camera_path
        self.quad_path = "/World/quadrotor/body"

        # Get the quadrotor prim to set initial camera position
        quad_prim = self.world.stage.GetPrimAtPath(self.quad_path)

        # Set initial position and rotation
        camera_xform = UsdGeom.Xformable(camera.GetPrim())

        if quad_prim:
            # Get initial quadrotor position and orientation
            quad_xform = UsdGeom.Xformable(quad_prim)
            try:
                quad_world_transform = quad_xform.ComputeLocalToWorldTransform(0)
                quad_position = quad_world_transform.ExtractTranslation()

                # Extract quadrotor rotation to get yaw
                quad_rotation = quad_world_transform.ExtractRotationQuat()

                # Extract yaw directly from quaternion components
                qw = quad_rotation.GetReal()
                qx, qy, qz = quad_rotation.GetImaginary()

                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                quad_yaw = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi

                # Calculate offset in the quadrotor's coordinate system
                offset_distance = 2.0
                offset_height = 0.5

                # Calculate offset based on quadrotor's yaw
                offset_x = -offset_distance * math.cos(math.radians(quad_yaw))
                offset_y = -offset_distance * math.sin(math.radians(quad_yaw))

                # Set initial camera position behind the quadrotor in its coordinate system
                camera_position = Gf.Vec3d(
                    quad_position[0] + offset_x,
                    quad_position[1] + offset_y,
                    quad_position[2] + offset_height,
                )

                camera_xform.AddTranslateOp().Set(camera_position)

                # # Add initial rotation
                camera_xform.AddRotateXYZOp().Set(Gf.Vec3d(85.0, 0.0, -90.0))


            except Exception as e:
                print(f"Could not set initial camera position: {e}")
                # Fall back to default position
                camera_xform.AddTranslateOp().Set(Gf.Vec3d(0, -5, 2))
                camera_xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, 0))
        else:
            # Default position if quadrotor not found
            camera_xform.AddTranslateOp().Set(Gf.Vec3d(0, -5, 2))
            camera_xform.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, 0))

        # Try to make this the active camera (but don't worry if it fails)
        try:
            import omni.ui
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            viewport.camera_path = camera_path
        except Exception as e:
            print(f"Note: Could not set active camera: {e}")
            print(
                "You may need to manually select the camera '/World/SimpleFollowCamera' in the viewport"
            )

        print("Camera follow system initialized (with look-at functionality)")


    def update_simple_camera(self):
        """
        Camera update that follows the quadrotor and points at it
        """
        from pxr import UsdGeom, Gf
        import math
        # Get the stage
        stage = self.world.stage
        # Get the quadrotor and camera prims
        quad_prim = stage.GetPrimAtPath(self.quad_path)
        camera_prim = stage.GetPrimAtPath(self.camera_path)
        if not quad_prim or not camera_prim:
            return
        # Get quadrotor position and orientation
        quad_xform = UsdGeom.Xformable(quad_prim)
        quad_world_transform = quad_xform.ComputeLocalToWorldTransform(0)
        quad_position = quad_world_transform.ExtractTranslation()
        # Extract quadrotor rotation as quaternion from transform matrix
        quad_rotation = quad_world_transform.ExtractRotationQuat()
        # Extract yaw directly from quaternion components
        # Yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        qw = quad_rotation.GetReal()
        qx, qy, qz = quad_rotation.GetImaginary()
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        quad_yaw = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi
        # Calculate the offset vector based on quadrotor's yaw
        offset_distance = 2.0  # Distance behind the quadrotor
        offset_height = 0.5  # Height above the quadrotor
        # Calculate offset in the quadrotor's coordinate system (rotated by yaw)
        offset_x = -offset_distance * math.cos(math.radians(quad_yaw))
        offset_y = -offset_distance * math.sin(math.radians(quad_yaw))
        # Calculate camera position (behind the quadrotor in its coordinate system)
        camera_position = Gf.Vec3d(
            quad_position[0] + offset_x,
            quad_position[1] + offset_y,
            quad_position[2] + offset_height,  # Up is always in Z regardless of rotation
        )
        # Update camera position
        camera_xform = UsdGeom.Xformable(camera_prim)
        # Get existing transform ops or create new ones
        ops = camera_xform.GetOrderedXformOps()
        # Update or create translate op
        translate_op = None
        rotate_op = None
        
        # Find existing ops
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translate_op = op
            elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                rotate_op = op
        
        # Update or create translate op
        if translate_op:
            translate_op.Set(camera_position)
        else:
            camera_xform.AddTranslateOp().Set(camera_position)
        
        # Update or create rotation op - combine fixed rotation with quadrotor's yaw
        if rotate_op:
            # Get current fixed rotation and update the Z component with quadrotor's yaw
            fixed_rotation = rotate_op.Get()
            # Add quad_yaw to the Z component (-90 + quad_yaw)
            rotate_op.Set(Gf.Vec3d(85.0, 0.0, -90.0 + quad_yaw))
        else:
            # Create new rotation with fixed rotation plus quadrotor's yaw
            camera_xform.AddRotateXYZOp().Set(Gf.Vec3d(85.0, 0.0, -90.0 + quad_yaw))
            

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            self.update_simple_camera()

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    # Instantiate the template app with the PX4 directory from command-line arguments
    pg_app = PegasusApp(args.px4_dir)

    # Run the application loop
    pg_app.run()


if __name__ == "__main__":
    main()
