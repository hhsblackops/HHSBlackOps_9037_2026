import rev
import math

from wpilib import AnalogEncoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
import constants
from typing import Tuple
from helpers.custom_hid import CustomHID

class SwerveModule:
    
    def __init__(self, dm_id: int, sm_id: int, analog_channel: int, steer_offset: float, drive_invert: bool, steer_invert: bool):
        print(f"Creating SwerveModule with drive CAN {dm_id}, steer CAN {sm_id}, enc CAN {analog_channel}, steer_offset {steer_offset}")
        """
        dm_id: Drive motor CAN ID, int.
        sm_id: Steer motor CAN ID, int.
        enc_id: Absolute encoder CAN ID, int.
        mod_offset: For Phoenix5 CANCoder setup, float. <- Scheduled to be deprecated. (We renamed it to steer_offset)
        drive_invert: Boolean for inverting drive motor, True/False.
        steer_invert: Boolean for inverting steer motor, True/False.
        """

        # create motors, encoders, and pid loops
        self.drive_motor = rev.SparkFlex(dm_id, rev.SparkFlex.MotorType.kBrushless)
        self.steer_motor = rev.SparkMax(sm_id, rev.SparkMax.MotorType.kBrushless)
        self.drive_encoder = self.drive_motor.getEncoder()
        self.steer_encoder = self.steer_motor.getEncoder()
        self.absolute_encoder = AnalogEncoder(analog_channel)
        self.steer_offset = steer_offset
       
        self.drive_pid = self.drive_motor.getClosedLoopController()
        self.steer_pid = self.steer_motor.getClosedLoopController()

        '''
        to adjust the configurations of a motor controller, you can no longer
        just pass them to to motor controller itself, you must create a seperate
        instance of a config, adjust the configs and then apply them using
        .configure() This is what happens below
        '''

        # create motor configs
        drive_motor_config = rev.SparkFlexConfig()
        steer_motor_config = rev.SparkMaxConfig()

        # adjust drive motor config
        drive_motor_config.encoder.velocityConversionFactor(constants.DriveConstants.d_velocity_conversion_factor)
        drive_motor_config.encoder.positionConversionFactor(constants.DriveConstants.d_position_conversion_factor)
        drive_motor_config.inverted(drive_invert)
        drive_motor_config.closedLoopRampRate(constants.DriveConstants.closed_loop_ramp)
        drive_motor_config.openLoopRampRate(constants.DriveConstants.open_loop_ramp)
        drive_motor_config.smartCurrentLimit(constants.DriveConstants.drive_current_limit)
        drive_motor_config.setIdleMode(rev.SparkFlexConfig.IdleMode.kBrake)
        drive_motor_config.closedLoop.pidf(constants.DriveConstants.ob_drive_pid[0], constants.DriveConstants.ob_drive_pid[1], constants.DriveConstants.ob_drive_pid[2], constants.DriveConstants.ob_drive_pid[3])

        # adjust steer motor config
        steer_motor_config.encoder.positionConversionFactor(1 / 12.8)
        steer_motor_config.absoluteEncoder.zeroOffset(steer_offset) #This shouldn't be causing any problems as far as errors go. Text me(Mordecai) the exact error or take a screenshot and plug it into chat. Let's see what we get because I bet that this isn't what's causing the error at least as far as syntax goes. The only issues I can think of it causing are with the argument variable. Maybe doa little checking on it. Good luck
        steer_motor_config.absoluteEncoder.zeroCentered(True) #This puts it in zero-centered mode shouldn't cause an error and should be within the library assuming it was downloaded correctly and you have a fairly recent version of the library
        steer_motor_config.inverted(steer_invert)
        steer_motor_config.closedLoopRampRate(constants.DriveConstants.closed_loop_ramp)
        steer_motor_config.openLoopRampRate(constants.DriveConstants.open_loop_ramp)
        steer_motor_config.smartCurrentLimit(constants.DriveConstants.azimuth_current_limit)
        steer_motor_config.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        steer_motor_config.closedLoop.pidf(constants.DriveConstants.ob_steer_pid[0], constants.DriveConstants.ob_steer_pid[1], constants.DriveConstants.ob_steer_pid[2], constants.DriveConstants.ob_steer_pid[3])

        self.set_relative_start()

        # apply motor configs
        self.drive_motor.configure(drive_motor_config, rev.SparkFlex.ResetMode.kResetSafeParameters, rev.SparkFlex.PersistMode.kPersistParameters)
        self.steer_motor.configure(steer_motor_config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        #with open("home/lvuser/offsetlog.txt", "a") as log:
        #    log.writelines(f"Creating SwerveModule with drive CAN {dm_id}, steer CAN {sm_id}, enc CAN {analog_channel}")
            
        


    def degree_to_steer(self, angle: Rotation2d) -> float:
        return (angle.degrees() % 360.0) / 360.0
    
    def set_desired_state_onboard(self, desired_state: SwerveModuleState):
        """Set a desired swerve module state for SPARK MAXes."""
        state = self.optimize_onboard(desired_state)

        if 0 < self.degree_to_steer(state.angle) <= 0.25 and 0.75 <= self.steer_encoder.getPosition() % 1 < 1:
            wrap_add = 1
        elif 0 < self.steer_encoder.getPosition() % 1 <= 0.25 and 0.75 <= self.degree_to_steer(state.angle) < 1:
            wrap_add = -1
        else:
            wrap_add = 0

        angle_mod = self.degree_to_steer(state.angle) + math.trunc(self.steer_encoder.getPosition()) + wrap_add
        self.drive_pid.setReference(state.speed, rev.SparkFlex.ControlType.kVelocity)
        self.steer_pid.setReference(angle_mod, rev.SparkMax.ControlType.kPosition)

    def reset_encoders(self):
        """Reset the drive encoder to its zero position."""
        self.drive_encoder.setPosition(0)

    def set_relative_start(self):

        print(f"{self.absolute_encoder.get()}")
        
        abs_pos = self.absolute_encoder.get() * 360

        angle_deg = abs_pos

        #with open("home/lvuser/offsetlog.txt", "a") as log:
        #    log.writelines(f"{self.absolute_encoder} Module absolute angle: {angle_deg:.2f} \n")
        print(f"{self.absolute_encoder} Module absolute angle: {angle_deg:.2f}")
        #print(f"Creating SwerveModule with drive CAN {dm_id}, steer CAN {sm_id}, enc CAN {analog_channel}")

        angle_deg -= self.steer_offset * 360
        angle_deg = angle_deg % 360

        relative_position = angle_deg / 360.0

        self.steer_encoder.setPosition(relative_position)

    def get_current_draw(self) -> Tuple[float, float]:
        """Returns a list of the drive and steering motor current draws."""
        return [self.drive_motor.getOutputCurrent(), self.steer_motor.getOutputCurrent()]

    def get_state_onboard(self) -> SwerveModuleState:
        """Returns the current swerve module state."""
        return SwerveModuleState(self.drive_encoder.getVelocity(),
                                 Rotation2d((self.steer_encoder.getPosition() % 1) * math.pi * 2))

    def get_position_onboard(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drive_encoder.getPosition(),
                                Rotation2d((self.steer_encoder.getPosition() % 1) * math.pi * 2))

    def optimize_onboard(self, desired_state: SwerveModuleState):
        inverted = False
        desired_degrees = desired_state.angle.degrees()
        if desired_degrees < 0:
            desired_degrees += 360  # converts desired degrees to 360

        current_degrees = (self.steer_encoder.getPosition() % 1) * 360  # converts current to 360

        if 90.0 < abs(current_degrees - desired_degrees) <= 270.0:
            inverted = True
            if desired_degrees > 180:
                desired_degrees -= 180
            else:
                desired_degrees += 180

        magnitude = desired_state.speed

        if inverted:
            magnitude *= -1

        return SwerveModuleState(magnitude, Rotation2d.fromDegrees(desired_degrees))
