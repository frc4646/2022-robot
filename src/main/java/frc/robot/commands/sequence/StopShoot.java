package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shooter.ShooterOpenLoop;

public class StopShoot extends ParallelCommandGroup {
  public StopShoot() {
    addCommands(
      new ShooterOpenLoop(0.0)
    );
  }
}
