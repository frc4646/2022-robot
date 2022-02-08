package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.agitator.AgitatorAuto;
import frc.robot.commands.feeder.FeederHasBall;
import frc.robot.commands.feeder.FeederOpenLoop;

public class LoadCargo extends SequentialCommandGroup {
  public LoadCargo() {
    addCommands(
      new ParallelDeadlineGroup(
        new FeederHasBall().withTimeout(5.0),
        new AgitatorAuto(.5)//,
        // new FeederOpenLoop(0.15)
      ),
      new ParallelCommandGroup(
        new AgitateOpenLoop(0.0),
        new FeederOpenLoop(0.0)
      )
    );
  }
}
