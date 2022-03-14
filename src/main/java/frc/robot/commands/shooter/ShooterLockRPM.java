package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/** Second cargo cannot interfere */
public class ShooterLockRPM extends InstantCommand {
  private final Shooter subsystem = RobotContainer.SHOOTER;
  private final Vision vision = RobotContainer.VISION;

  public ShooterLockRPM() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    double setpoint = vision.getShooterRPM();
    subsystem.setClosedLoop(setpoint);
  }
}
