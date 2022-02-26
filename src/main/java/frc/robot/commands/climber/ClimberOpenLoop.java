package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberOpenLoop extends InstantCommand {
  private Climber subsystem = RobotContainer.CLIMBER;
  private final double output;

  public ClimberOpenLoop(double percent) {
    addRequirements(subsystem);
    output = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(output);
  }
}
