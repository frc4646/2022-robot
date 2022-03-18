package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;

public class AgitateOpenLoop extends InstantCommand {
  private final Agitator subsystem = RobotContainer.AGITATOR;
  private final double output;

  public AgitateOpenLoop(double percent) {
    addRequirements(subsystem);
    output = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(output, output);
  }
}
