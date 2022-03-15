package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterTop;

public class ShooterTopVelocity extends InstantCommand {
  private ShooterTop subsystem = RobotContainer.SHOOTER_TOP;
  private final double rpm;

  public ShooterTopVelocity(double rpm) {
    addRequirements(subsystem);
    this.rpm = rpm;
  }

  @Override
  public void initialize() {
    subsystem.setClosedLoop(rpm);
  }
}
