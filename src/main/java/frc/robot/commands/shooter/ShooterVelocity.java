package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterVelocity extends CommandBase {
  private Shooter subsystem = RobotContainer.SHOOTER;

  private final double output;

  public ShooterVelocity(double rpm) {
    addRequirements(subsystem);
    this.output = rpm;
  }

  @Override
  public void initialize() {
    subsystem.setClosedLoopVelocity(output);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isOnTarget();
  }
}
