import commands2
import math
from commands2 import button
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.limelight_camera import LimelightCamera
from wpilib import DriverStation, Timer
from helpers.custom_hid import CustomHID
from helpers.pose_estimator import PoseEstimator



class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.timer = Timer()
        self.timer.start()
        self.pose_estimator = PoseEstimator()
        self.robot_drive = DriveSubsystem(self.timer, self.pose_estimator)
        self.camera = LimelightCamera("limelight-pickup")

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(0, "xbox")

        # Perform setup as normal, unless tuning mode is enabled.
    
        self.robot_drive.setDefaultCommand(commands2.cmd.run(
            lambda: self.robot_drive.drive_2ok(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * -5.06,
                #TRYING THIS AS A NEGATIVE TO SEE WHAT HAPPENS
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * 5.06,
                self.driver_controller_raw.get_axis_squared("RX", 0.06) * (7 * 2 * math.pi),
                True
            ), self.robot_drive
        ))

        # Setup for all event-trigger commands.
        self.configureTriggersDefault()


    def configureTriggersDefault(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        button.Trigger(lambda: DriverStation.isTeleopEnabled()).onTrue(
            commands2.cmd.runOnce(lambda: self.robot_drive.set_alliance(), self.robot_drive))
        button.Trigger(lambda: DriverStation.isDSAttached()).onTrue(
            commands2.cmd.runOnce(lambda: self.robot_drive.set_alliance(), self.robot_drive))

        # Hold for Parking Brake.
        # button.Trigger(lambda: self.driver_controller_raw.get_trigger("L", 0.05)).whileTrue(
        #     commands2.cmd.run(lambda: self.robot_drive.drive_lock(), self.robot_drive))


        # Hold for Slow Mode, variable based on depth of Trigger.
        #button.Trigger(lambda: self.driver_controller_raw.get_trigger("R", 0.05)).whileTrue(
        #    commands2.cmd.run(lambda: self.robot_drive.drive_slow(
        #        self.driver_controller_raw.get_axis_squared("LY", 0.06) * 5.06,
        #        self.driver_controller_raw.get_axis_squared("LX", 0.06) * 5.06,
        #        self.driver_controller_raw.get_axis("RX", 0.06),
        #        True,
        #        self.driver_controller_raw.refine_trigger("R", 0.05, 0.8, 0.3)), self.robot_drive))

        # Press any direction on the D-pad to enable PID snap to that equivalent angle based on field orientation
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * 5.06,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * 5.06,
                180
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * 5.06,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * 5.06,
                0
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * 5.06,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * 5.06,
                60
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * 5.06,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * 5.06,
                300
            ), self.robot_drive))
        """
        button.Trigger(lambda: self.driver_controller_raw.get_button("A")).whileTrue(
            x = self.camera.getX()
            print(f"CameraX: {x}")
            turn_speed = -0.005 * x

            )
        """
    