package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RobotState extends SmartSubsystem {  
  private final double GAIN_YAW_RATE = 3.0;  // TODO test non-zero
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final Feeder feeder = RobotContainer.FEEDER;
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final Turret turret = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;

  @Override
  public void updateDashboard(boolean showDetails) {
    if (Constants.TUNING.ROBOT_STATE) {
      SmartDashboard.putNumber("State: Predicted Yaw", getDrivePredictedYaw());
      SmartDashboard.putNumber("State: Turret Vision", getTurretVisionSetpoint());
      SmartDashboard.putNumber("State: Turret Fused", getTurretFusedSetpoint());
    }
  }

  public double getDrivePredictedYaw() {
    return drive.getHeadingRate() * .02;
  }

  public double getTurretVisionSetpoint() {
    return turret.getPosition() + vision.getTurretError() * Constants.TURRET.GAIN_STABILITY;
  }

  public double getTurretFusedSetpoint() {
    return getTurretVisionSetpoint() + getDrivePredictedYaw() * GAIN_YAW_RATE;
  }

  public boolean isShootingStable() {
    return shooter.isStable() && shooterTop.isStable();
  }

  public boolean isCanShoot() {
    return isShootingStable() && turret.isOnTarget() && vision.isStable();
  }  

  public boolean isAutoRevWanted() {
    return feeder.isShooterLoaded() && feeder.isHooperLoaded() && vision.isTargetPresent();
  }
}
