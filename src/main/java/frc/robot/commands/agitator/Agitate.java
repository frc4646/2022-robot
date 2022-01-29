package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;

public class Agitate extends InstantCommand {
  private Agitator subsystem = RobotContainer.AGITATOR;

  private final double output;

  public Agitate(double percent) {
    addRequirements(subsystem);
    output = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(output);
  }
}
