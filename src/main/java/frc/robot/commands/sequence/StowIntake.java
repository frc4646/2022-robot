package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;

public class StowIntake extends SequentialCommandGroup {
  public StowIntake() {
    addCommands(
      new IntakeExtend(false),
      new IntakeActivate(0.0),
      new WaitCommand(Constants.AGITATOR.TIMEOUT_STOW),  // Let agitators settle cargo
      new AgitateOpenLoop(0.0)
    );
  }
}
