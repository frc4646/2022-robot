package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Vision;
import frc.robot.util.ShootSetpoint;

/** Second cargo cannot interfere */
public class ShooterLockRPM extends InstantCommand {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final Vision vision = RobotContainer.VISION;

  public ShooterLockRPM() {
    addRequirements(shooter, shooterTop);
  }

  @Override
  public void initialize() {
    ShootSetpoint setpoint = vision.getShooterRPM();
    shooter.setClosedLoop(setpoint, true);
    shooterTop.setClosedLoop(setpoint);
  }
}
