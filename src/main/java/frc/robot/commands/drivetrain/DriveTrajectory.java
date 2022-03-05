package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrajectory extends SequentialCommandGroup {
  /** Add your docs here. */
  public DriveTrajectory(Trajectory trajectory) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    addCommands(
      new RamseteCommand(
        trajectory,
        RobotContainer.DRIVETRAIN::getPose,
        new RamseteController(Constants.DRIVETRAIN.RAMSETE_B, Constants.DRIVETRAIN.RAMSETE_ZETA),
        Constants.DRIVETRAIN.FEED_FORWARD,
        Constants.DRIVETRAIN.DRIVE_KINEMATICS,
        RobotContainer.DRIVETRAIN::getWheelSpeeds,
        // TODO Do we want the ramsete volts or velocity constructor?
        new PIDController(Constants.DRIVETRAIN.P_LEFT, Constants.DRIVETRAIN.I_LEFT, Constants.DRIVETRAIN.D_LEFT),
        new PIDController(Constants.DRIVETRAIN.P_RIGHT, Constants.DRIVETRAIN.I_RIGHT, Constants.DRIVETRAIN.D_RIGHT),
        RobotContainer.DRIVETRAIN::setVolts,
        RobotContainer.DRIVETRAIN
      )
    );
    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
