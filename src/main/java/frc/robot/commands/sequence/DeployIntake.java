package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitatorPulse;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;

public class DeployIntake extends SequentialCommandGroup {
  public DeployIntake() {
    addCommands(
      new IntakeExtend(true),
      new IntakeActivate(Constants.INTAKE.OPEN_LOOP),
      new AgitatorPulse(Constants.AGITATOR.OPEN_LOOP_LOAD * 1.5, 0.5)
    );
  }
}
