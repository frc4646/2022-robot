package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;

public class DeployIntake extends SequentialCommandGroup {
  public DeployIntake() {
    addCommands(
      new IntakeExtend(true),
      new IntakeActivate(1.0)
    );
  }
}
