package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DrivePath extends SequentialCommandGroup {
  public DrivePath(Trajectory trajectory) {
    this(trajectory, true);
  }

  public DrivePath(Trajectory trajectory, boolean velocityMode) {
    addCommands(velocityMode ? ramseteVelocity(trajectory) : ramseteVolts(trajectory));
  }

  /** PID calculated on motor controller */
  private RamseteCommand ramseteVelocity(Trajectory trajectory) {
    return new RamseteCommand(
      trajectory,
      RobotContainer.DRIVETRAIN::getPose,
      new RamseteController(Constants.DRIVETRAIN.RAMSETE_B, Constants.DRIVETRAIN.RAMSETE_ZETA),  // TODO make configurable?
      Constants.DRIVETRAIN.KINEMATICS,
      RobotContainer.DRIVETRAIN::setClosedLoopVelocity,
      RobotContainer.DRIVETRAIN
    );
  }

  /** PID calculated on roboRIO */
  private RamseteCommand ramseteVolts(Trajectory trajectory) {
    return new RamseteCommand(
      trajectory,
      RobotContainer.DRIVETRAIN::getPose,
      new RamseteController(Constants.DRIVETRAIN.RAMSETE_B, Constants.DRIVETRAIN.RAMSETE_ZETA),
      Constants.DRIVETRAIN.FEED_FORWARD,
      Constants.DRIVETRAIN.KINEMATICS,
      RobotContainer.DRIVETRAIN::getWheelSpeeds,
      new PIDController(Constants.DRIVETRAIN.VOLTAGE_P, Constants.DRIVETRAIN.VOLTAGE_I, Constants.DRIVETRAIN.VOLTAGE_D),
      new PIDController(Constants.DRIVETRAIN.VOLTAGE_P, Constants.DRIVETRAIN.VOLTAGE_I, Constants.DRIVETRAIN.VOLTAGE_D),
      RobotContainer.DRIVETRAIN::setVolts,
      RobotContainer.DRIVETRAIN
    );
  }
}
