package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodOpenLoop extends InstantCommand {
  private final Hood subsystem = RobotContainer.HOOD;

  private final double percent;

  public HoodOpenLoop(double percent) {
    addRequirements(subsystem);
    this.percent = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(percent);
  }
}
