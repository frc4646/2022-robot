package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.commands.shooter.ShooterTune;

public class TuneInterpolation extends SequentialCommandGroup {
  public TuneInterpolation() {
    addCommands(
      new ShooterTune(),
      new FireCargo(),
      new ShooterOpenLoop(0.0)
    );
  }
}
