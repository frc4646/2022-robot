package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Vision;

/** Second cargo cannot interfere */
public class ShooterTopLockRPM extends InstantCommand {
  private final ShooterTop subsystem = RobotContainer.SHOOTER_TOP;
  private final Vision vision = RobotContainer.VISION;

  public ShooterTopLockRPM() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    double setpoint = vision.getShooterRPMTop();
    subsystem.setClosedLoop(setpoint);
  }
}
