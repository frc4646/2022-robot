package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrajectory extends SequentialCommandGroup {
  /** Add your docs here. */
  public DriveTrajectory(TrajectoryConfig trajectory) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    addCommands(
      new RamseteCommand(
        trajectory,
        RobotContainer.DRIVETRAIN::getPose,
        new RamseteController(Constants.Drivetrain.RAMSETE_B, Constants.Drivetrain.RAMSETE_ZETA),
        new SimpleMotorFeedforward(
          Constants.Drivetrain.FEED_FORWARD_GAIN_STATIC,
          Constants.Drivetrain.FEED_FORWARD_GAIN_VELOCITY,
          Constants.Drivetrain.FEED_FORWARD_GAIN_ACCEL
        ),
        Constants.Drivetrain.DRIVE_KINEMATICS,
        RobotContainer.DRIVETRAIN::getWheelSpeeds,
        // TODO Do we want the ramsete volts or velocity constructor?
        new PIDController(Constants.Drivetrain.P_LEFT, Constants.Drivetrain.I_LEFT, Constants.Drivetrain.D_LEFT),
        new PIDController(Constants.Drivetrain.P_RIGHT, Constants.Drivetrain.I_RIGHT, Constants.Drivetrain.D_RIGHT),
        outputVolts,  // TODO why does RobotContainer.DRIVETRAIN::setVolts not work???
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
