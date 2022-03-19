package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Vision;
import frc.robot.util.ShootSetpoint;

public class ShooterRev extends CommandBase {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final Vision vision = RobotContainer.VISION;
  private final ShootSetpoint setpointNoVision;

  public ShooterRev() {
    this(Constants.VISION.MAP.getDistanceDefault());
  }

  public ShooterRev(double distance) {
    addRequirements(shooter, shooterTop);
    setpointNoVision = new ShootSetpoint(distance);
  }

  @Override
  public void execute() {
    ShootSetpoint setpoint = vision.isTargetPresent() ? vision.getShooterRPM() : setpointNoVision;
    shooter.setClosedLoop(setpoint, false);
    shooterTop.setClosedLoop(setpoint);
  }
}
