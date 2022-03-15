package frc.robot.commands.shooterTop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterTop;

public class ShooterTopOpenLoop extends InstantCommand {
  private ShooterTop subsystem = RobotContainer.SHOOTER_TOP;
  public final double output;

  public ShooterTopOpenLoop(double percent) {
    addRequirements(subsystem);
    this.output = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(output);
  }
}
