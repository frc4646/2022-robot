package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.indexer.IndexOpenLoop;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.commands.shooter.ShooterTune;

public class ShootOpenLoop extends SequentialCommandGroup {
  public ShootOpenLoop() {
    addCommands(
      new ShooterTune(),
      new WaitCommand(Constants.Shooter.REV_TIME),
      new IndexOpenLoop(.5),
      new WaitCommand(2.0),
      new ShooterOpenLoop(0.0),
      new IndexOpenLoop(0.0)
    );
  }
}
