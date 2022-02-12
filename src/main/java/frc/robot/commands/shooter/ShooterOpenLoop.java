package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterOpenLoop extends InstantCommand {
  public Shooter subsystem = RobotContainer.SHOOTER;
  public final double output;

  public ShooterOpenLoop(double percent) {
    addRequirements(subsystem);
    this.output = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(output);
  }
}
