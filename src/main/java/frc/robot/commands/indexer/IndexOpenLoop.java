package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;

public class IndexOpenLoop extends InstantCommand {
  private Indexer subsystem = RobotContainer.INDEXER;

  private final double output;

  public IndexOpenLoop(double percent) {
    addRequirements(subsystem);
    output = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(output);
  }
}
