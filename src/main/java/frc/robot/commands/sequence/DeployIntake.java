package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeExtend;

public class DeployIntake extends ParallelCommandGroup {
  public DeployIntake() {
    addCommands(
      new IntakeExtend(true),
      new IntakeOpenLoop(Constants.INTAKE.OPEN_LOOP).perpetually()
    );
  }
}
