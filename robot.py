from commands2 import Command, CommandScheduler, TimedCommandRobot
from subsystems.robotcontainer import RobotContainer
from wpilib import run, Timer


class Robot(TimedCommandRobot):
    """This class allows the programmer to control what runs in each individual robot operation mode."""
    m_autonomous_command: Command  # Definition for autonomous command groups used in autonomousInit
    m_robotcontainer: RobotContainer  # Type-check for robotcontainer class
    # m_timer = wpilib..timer()

    def robotInit(self) -> None:
        """Initialize the robot through the RobotContainer object and prep the default autonomous command (None)"""
        # CameraServer.launch()
        self.m_robotcontainer = RobotContainer()
        self.m_autonomous_command = None
        self.m_timer = Timer()
        self.first_run = False


    def robotPeriodic(self) -> None:
        """Set the constant robot periodic state (in command based, that's just run the scheduler loop)"""
        CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        """Nothing is written here yet. Probably will not modify unless something is required for end-of-match."""

    def disabledPeriodic(self) -> None:
        """This isn't the most useful state to call anything in because you can set commands to run in disabled.
        So it's not really anything at all right now."""

    def autonomousInit(self) -> None:
        """Run the auto scheduler if the command was actually input. For the most part, this is a safety call."""
        if self.m_autonomous_command is not None:
            self.m_autonomous_command.schedule()


    def autonomousPeriodic(self) -> None:
        """Empty for now. Handled by command scheduler."""
        """
        if (self.m_timer.get() < 2.0) and (self.first_run == False):
            self.m_robotcontainer.robot_drive.drive_2ok(0, 0.1, 0, True)
            self.first_run = True
        elif self.m_timer.get() >= 2.0:
            self.m_robotcontainer.robot_drive.drive_2ok(0, 0, 0, True)
        """

    def teleopInit(self) -> None:
        """Shuts off the auto command if one is being run. Could be altered to allow the command to proceed into
        teleop mode."""
        if self.m_autonomous_command:
            self.m_autonomous_command.cancel()
        #self.m_robotcontainer.leds.set_state("default")
        

    def teleopPeriodic(self) -> None:
        """Nothing relevant here yet, everything's covered by the master scheduler."""

    def testInit(self) -> None:
        """Reset the scheduler automatically when entering test mode."""
        CommandScheduler.getInstance().cancelAll()

    def _simulationPeriodic(self) -> None:
        """Empty for now as well."""

if __name__ == "__main__":
    run(Robot)