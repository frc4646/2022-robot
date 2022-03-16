package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/** Second cargo cannot interfere */
public class TurretLockPosition extends InstantCommand {
  private final Turret turret = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;

  public TurretLockPosition() {
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    double setpoint = turret.getPosition() - vision.getTurretSetpoint();
    turret.setSetpointMotionMagic(setpoint, 0.0);
  }
}
