package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootExhaust extends SequentialCommandGroup {
  public ShootExhaust() {
    addCommands(
      new PrintCommand("TODO: Shoot Exhaust")
    );
  }
}
