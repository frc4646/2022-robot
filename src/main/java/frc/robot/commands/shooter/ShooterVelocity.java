package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterVelocity extends CommandBase {
  private Shooter subsystem = RobotContainer.SHOOTER;

  private final int output;

  public ShooterVelocity(int velocity) {
    addRequirements(subsystem);
    this.output = velocity;
  }

  @Override
  public void initialize() {
    subsystem.setSpeed(output);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isOnTarget();
  }
}
