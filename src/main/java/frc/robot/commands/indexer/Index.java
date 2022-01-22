package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;

public class Index extends InstantCommand {
  private Indexer subsystem = RobotContainer.INDEXER;
double output; 
  public Index(double percent) {
    addRequirements(subsystem);
    output = percent;
  }
  @Override
  public void initialize() {
    subsystem.setIndex(output);
  }


}
