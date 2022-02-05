package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederHasBall;
import frc.robot.commands.feeder.FeederOpenLoop;

public class LoadCargo extends SequentialCommandGroup {
  public LoadCargo() {
    addCommands(
      new ParallelDeadlineGroup(
        new FeederHasBall().withTimeout(2.0),  // Load even if sensor broken using timeout
        new AgitateOpenLoop(0.5),
        new FeederOpenLoop(0.1)
      ),
      new ParallelCommandGroup(
        new AgitateOpenLoop(0.0),
        new FeederOpenLoop(0.0)
      )
    );
  }
}
