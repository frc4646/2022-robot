package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.team254.util.DriveSignal;
import frc.team254.util.OpenLoopCheesyDriveHelper;

public class DriveTeleop extends CommandBase {
  private final Drivetrain subsystem = RobotContainer.DRIVETRAIN;
  private final DriverControls controls = RobotContainer.CONTROLS.getDriver();
  private final OpenLoopCheesyDriveHelper steeringController = OpenLoopCheesyDriveHelper.getInstance();
  private final SlewRateLimiter throttleAccelLimiter = new SlewRateLimiter(Constants.DRIVETRAIN.THROTTLE_SLEW_LIMIT);
  private final Shooter shooter = RobotContainer.SHOOTER;

  public DriveTeleop() {
    addRequirements(subsystem);
  }

  public void initialize() {
    subsystem.setBrakeMode(false);
  }

  @Override
  public void execute() {
    double stickThrottle = controls.getThrottle();
    if (shooter.isShooting()) {
      stickThrottle = stickThrottle * 0.05;
    }
    final double throttle = throttleAccelLimiter.calculate(stickThrottle);
    final DriveSignal output = steeringController.cheesyDrive(throttle, controls.getTurning(), controls.getQuickturn());
    subsystem.setOpenLoop(output.getLeft(), output.getRight());
  }
}
