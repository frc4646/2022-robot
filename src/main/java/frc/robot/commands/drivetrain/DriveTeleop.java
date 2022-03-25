package frc.robot.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.DriverControls;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.team254.util.DriveSignal;
import frc.team254.util.OpenLoopCheesyDriveHelper;

public class DriveTeleop extends CommandBase {
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final DriverControls controls = RobotContainer.CONTROLS.getDriver();
  private final OpenLoopCheesyDriveHelper steeringController = OpenLoopCheesyDriveHelper.getInstance();
  private final SlewRateLimiter throttleAccelLimiter = new SlewRateLimiter(Constants.DRIVETRAIN.THROTTLE_SLEW_LIMIT);
  private final Climber climber = RobotContainer.CLIMBER;
  private final Shooter shooter = RobotContainer.SHOOTER;

  public DriveTeleop() {
    addRequirements(drive);
  }

  public void initialize() {
    drive.setBrakeMode(false);
  }

  @Override
  public void execute() {
    double stickThrottle = controls.getThrottle();
    double stickTurn = controls.getTurning();
    if (shooter.isIntendingToShoot()) {
      DifferentialDriveWheelSpeeds speed = drive.getWheelSpeeds();
      double speedMetersPerSecond = (speed.leftMetersPerSecond + speed.rightMetersPerSecond) / 2.0;
      boolean safeToBrake = Math.abs(speedMetersPerSecond) < 0.25;
      stickThrottle = 0.0;
      stickTurn = 0.0;
      drive.setBrakeMode(safeToBrake);
    }
    //  else if (climber.isInClimbMode()) {
    //   stickThrottle *= 0.25;
    //   stickTurn *= 0.25;
    // } 
    else {
      drive.setBrakeMode(false);
    }
    final double throttle = throttleAccelLimiter.calculate(stickThrottle);
    final DriveSignal output = steeringController.cheesyDrive(throttle, stickTurn, controls.getQuickturn());
    drive.setOpenLoop(output.getLeft(), output.getRight());
  }
}
