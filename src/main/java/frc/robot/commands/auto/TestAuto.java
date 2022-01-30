package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.sequence.TuneInterpolation;
import frc.robot.commands.sequence.StopShoot;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto() {
    addCommands(
      new TuneInterpolation(),
      new StopShoot()
    );
  }
}
