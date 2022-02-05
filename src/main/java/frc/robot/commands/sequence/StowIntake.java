package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;

public class StowIntake extends SequentialCommandGroup {
  public StowIntake() {
    addCommands(
      new IntakeExtend(false),
      new WaitCommand(0.0),
      new IntakeActivate(0.0)
    );
  }
}
