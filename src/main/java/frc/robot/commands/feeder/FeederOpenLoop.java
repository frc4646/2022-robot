package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederOpenLoop extends InstantCommand {
  private final Feeder subsystem = RobotContainer.FEEDER;
  private final double output;
  
  public FeederOpenLoop() {
    this(0.0);
  }

  public FeederOpenLoop(double percent) {
    addRequirements(subsystem);
    output = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(output);
  }
}
