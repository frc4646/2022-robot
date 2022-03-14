package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterVelocity extends InstantCommand {
  private Shooter subsystem = RobotContainer.SHOOTER;
  private final double rpm;

  public ShooterVelocity(double rpm) {
    addRequirements(subsystem);
    this.rpm = rpm;
  }

  @Override
  public void initialize() {
    subsystem.setClosedLoop(rpm);
  }
}
