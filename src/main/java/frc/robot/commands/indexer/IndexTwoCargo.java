package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexTwoCargo extends CommandBase {
  private Indexer subsystem = RobotContainer.INDEXER;

  public IndexTwoCargo() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    // TODO ask how many cargo in indexer
    // TODO maybe figure out if subsystem is jammed and do something like pulse it in reverse?
    // TODO need timer to debounce?
    // TODO should we have a state machine?
    // TODO set motor
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setOpenLoop(0.0);
  }
}
