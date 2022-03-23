package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RobotState extends SmartSubsystem {
  private final ColorSensor colorSensor = RobotContainer.COLOR_SENSOR;
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final Feeder feeder = RobotContainer.FEEDER;
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final Turret turret = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;

  @Override
  public void updateDashboard(boolean showDetails) {
    if (Constants.TUNING.ROBOT_STATE) {
    }
  }

  public double getDrivePredictedYaw() {
    return drive.getHeadingRate() * .02;
  }

  public double getTurretVisionSetpoint() {
    return turret.getPosition() + vision.getTurretError() * Constants.TURRET.GAIN_STABILITY;
  }

  public boolean isShootingStable() {
    return shooter.isStable() && shooterTop.isStable();
  }

  public boolean isCanShoot() {
    return isShootingStable() && turret.isOnTarget() && vision.isStable();
  }  

  public boolean isAutoRevWanted() {
    return vision.isTargetPresent() && 
          (feeder.isHopperFull() && feeder.isCargoIndexed()) ||
          (feeder.isHopperFull() && feeder.isShooterLoaded()) ||
          (feeder.isCargoIndexed() && feeder.isShooterLoaded()) ;
  }

  public boolean isShootExhaustWanted() {
    return colorSensor.isWrongCargo() && feeder.isShooterLoaded();
  }

  // public double getTurretFusedFeedForward() {
  //   Rotation2d turretDegrees = Rotation2d.fromDegrees(turret.getPosition());
  //   Rotation2d hubDegrees = turretDegrees.rotateBy(Rotation2d.fromDegrees(vision.getTurretError()));
  //   Rotation2d dAngleDriveYawDegrees = Rotation2d.fromDegrees(drive.getHeadingRate());

  //   double dxParallelToHubMeters = hubDegrees.getSin() * drive.getPose().getX();
  //   double distanceToHubMeters = vision.groundDistanceToHubInches() / 0.0254;

  //   Rotation2d dAngleRobotToHubDegrees = Rotation2d.fromDegrees(dxParallelToHubMeters / distanceToHubMeters);
  //   Rotation2d errorDrivingInducedDegrees = dAngleDriveYawDegrees.rotateBy(dAngleRobotToHubDegrees);

  //   return -errorDrivingInducedDegrees.getDegrees();
  // }
}
