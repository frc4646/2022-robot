package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.Drivetrain;
import frc.team254.util.DriveSignal;
import frc.team254.util.OpenLoopCheesyDriveHelper;

public class DriveTeleop extends CommandBase {
  private final Drivetrain subsystem = RobotContainer.DRIVETRAIN;
  private final DriverControls controls = RobotContainer.CONTROLS.getDriver();
  private final OpenLoopCheesyDriveHelper steeringController = OpenLoopCheesyDriveHelper.getInstance();
  private final SlewRateLimiter throttleAccelLimiter = new SlewRateLimiter(Constants.DRIVETRAIN.THROTTLE_SLEW_LIMIT);

  public DriveTeleop() {
    addRequirements(subsystem);
  }

  public void initialize() {
    subsystem.setBrakeMode(false);
  }

  @Override
  public void execute() {
    // Using Slew Rate Limiter: See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html#using-a-slewratelimiter-with-differentialdrive
    // final double throttle = throttleAccelLimiter.calculate(controls.getThrottle());
    final double throttle = controls.getThrottle();
    final DriveSignal output = steeringController.cheesyDrive(throttle, controls.getTurning(), controls.getQuickturn());
    subsystem.setOpenLoop(output.getLeft(), output.getRight());
  }
}
