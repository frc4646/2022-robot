package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Vision;

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
    shooter.setClosedLoop(vision.getShooterRPMBottom(), true);
    shooterTop.setClosedLoop(vision.getShooterRPMTop());
  }
}
