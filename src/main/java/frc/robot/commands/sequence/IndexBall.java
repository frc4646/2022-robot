package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederHasBall;
import frc.robot.commands.feeder.FeederOpenLoop;

public class IndexBall extends SequentialCommandGroup {
  public IndexBall() {
    addCommands(
      new ParallelCommandGroup(
        new AgitateOpenLoop(0.5),
        new FeederOpenLoop(Constants.Feeder.PERCENT_OPEN_LOOP)
      ),
      new FeederHasBall(),
      new ParallelCommandGroup(
        new AgitateOpenLoop(0.0),
        new FeederOpenLoop(0.0)
      )
    );
  }
}
