package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;

public class DeployIntake extends SequentialCommandGroup {
  public DeployIntake() {
    addCommands(
      new IntakeExtend(true),
      new IntakeActivate(Constants.INTAKE.OPEN_LOOP),
      new AgitateOpenLoop(Constants.AGITATOR.OPEN_LOOP_LOAD)
    );
  }
}
