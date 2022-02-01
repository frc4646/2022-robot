package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.shooter.ShooterOpenLoop;

public class StopShoot extends ParallelCommandGroup {
  public StopShoot() {
    addCommands(
      new ShooterOpenLoop(0.0),
      new FeederOpenLoop(0.0),
      new AgitateOpenLoop(0.0)
    );
  }
}
