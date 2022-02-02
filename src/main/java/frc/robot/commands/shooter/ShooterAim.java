package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShooterAim extends CommandBase {
  private Shooter subsystem = RobotContainer.SHOOTER;
  private Vision vision = RobotContainer.VISION;

  public ShooterAim() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (vision.isTargetPresent()) {
      subsystem.setClosedLoop(vision.getShooterVelocityRPM());
    }
  }

  @Override
  public boolean isFinished() {
    return subsystem.isOnTarget();
  }
}
