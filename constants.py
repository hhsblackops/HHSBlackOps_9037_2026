"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""

#These all need to be adjusted as well for the new robot. -Tristan


import math
from pathplannerlib.config import ModuleConfig
from wpimath.system.plant import DCMotor
from wpimath.geometry import Translation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import feetToMeters

class DriveConstants:
    wheel_diameter = 0.1016  # meters
    wheel_circumference = wheel_diameter * math.pi
    drive_gear_ratio = 6.12
    angle_gear_ratio = 12.8
    #Was 21.43

    # d_velocity_conversion_factor = 0.0007885761
    # d_position_conversion_factor = 0.047314566  # L2 ratio is 6.746031745
    d_position_conversion_factor = wheel_circumference / drive_gear_ratio
    d_velocity_conversion_factor = d_position_conversion_factor / 60  # Conversion from rot/min to m/s
    kMaxSpeed = 5.06  # Set max speed in m/s 10
    # kMaxSpeedTeleop = kMaxSpeed * 1.75
    kMaxAngularSpeed = 7  # Set max rotation speed rot/s 20
    # kMaxAngularSpeedTeleop = kMaxAngularSpeed * 1.75
    kGyroReversed = False

    m_FL_location = Translation2d(0.52705 / 2, -0.52705 / 2)
    m_FR_location = Translation2d(0.52705 / 2, 0.52705 / 2)
    m_BL_location = Translation2d(-0.52705 / 2, -0.52705 / 2)
    m_BR_location = Translation2d(-0.52705 / 2, 0.52705 / 2)
    m_kinematics = SwerveDrive4Kinematics(m_FL_location, m_FR_location, m_BL_location, m_BR_location)

    snap_controller_PID = [0.051, 0, 0]  # 0.01
    turret_controller_PID = [0.08, 0, 0.0001]
    clt_controller_PID = [0.04, 0, 0]  # 11
    drive_controller_PID = [2, 0, 0]
    azimuth_controller_PID = [1.8, 0, 0]
    drive_controller_FF = [0.18 / 12, 2.35, 0.44]  # n/a, 2.35, 0.44

    closed_loop_ramp = 0.1  # was 0.0
    open_loop_ramp = 0.25
    drive_current_limit = 50  # was 60
    azimuth_current_limit = 20  # Was 30

    balance_PID = [0.01, 0, 0]

    slew_rate_drive = 130  # 50  # 110
    slew_rate_turn = 0

    # ob_drive_pid = [0, 0, 0, 1/16.6] need to convert to meters, bud
    ob_drive_pid = [0.5, 0, 0, 1 / feetToMeters(16.6)]
    ob_steer_pid = [1, 0, 0, 0]

class AutoConstants:
    kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeed
    kMaxAccelerationMetersPerSecondSquared = 0.5

    kPXController = 5.2  # Previously 5.2
    kIXController = 0
    kDXController = 0
    kPThetaController = 5.2  # Previously 5.2
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(kMaxSpeedMetersPerSecond,
                                                                      kMaxAccelerationMetersPerSecondSquared)
    max_module_speed = kMaxSpeedMetersPerSecond  # * 0.8
    module_distance = 0.52705  # in m
    # module_radius_from_center = math.sqrt(pow((module_distance / 2), 2) + pow((module_distance / 2), 2))
    module_radius_from_center = 0.372681


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1

class ModuleConstants:

    fr_drive_id = 8
    # Was 6
    fr_turn_id = 9
    # Was 4
    fr_encoder_id = 2
    fr_zero_offset = 58.36/360
    #2nd Time Gave out 205.95
    #Gave out 331.47
    #Was -223.33 on old robot
    #Was 53.53

    fl_drive_id = 2
    # Was 6
    fl_turn_id = 3
    # Was 16
    fl_encoder_id = 3
    fl_zero_offset = 171.82/360
    #2nd Time gave out 180.85
    #Gave out 177.69
    #Was -248.91 on old robot


    br_drive_id = 18
    # Was 5
    br_turn_id = 7
    # Was 3
    br_encoder_id = 1
    br_zero_offset = 42.50/360
    #2nd time gave us 318.87
    #It gave us 280.96
    #Was -53.53
    #Was -213.53

    bl_drive_id = 4
    # Was 8
    bl_turn_id = 5
    # Was 2
    bl_encoder_id = 0
    bl_zero_offset = 150.61/360
    #2nd Time Gave out 29.4
    #It gave us 280.96
    #Was -202.85

    module_config = ModuleConfig(
        0.0508, 
        4.9,
        1.0, 
        DCMotor.neoVortex(), 
        40, 
        1)

    half_width = 0.3
    half_length = 0.3

    module_offsets = [
        Translation2d(half_length,  half_width),   # Front Left
        Translation2d(half_length, -half_width),   # Front Right
        Translation2d(-half_length, half_width),   # Back Left
        Translation2d(-half_length, -half_width),  # Back Right
    ]

class WoodConstants:
    wood_intake_id = 14
    wood_intake_encoder_id = 14
    
class LovelyLauncherConstants:
    launcher_id = 10
    launcher_encoder_id = 10
class LovelyIntakeConstants:
    feeding_id = 11
    feeding_encoder_id = 11

