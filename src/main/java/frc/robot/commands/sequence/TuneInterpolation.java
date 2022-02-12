package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.shooter.ShooterTune;

public class TuneInterpolation extends SequentialCommandGroup {
  public TuneInterpolation() {
    addCommands(
      new ShooterTune(),
      new FeederOpenLoop(Constants.Feeder.OPEN_LOOP_PERCENT),
      new WaitCommand(1.0)
    );
  }
}
