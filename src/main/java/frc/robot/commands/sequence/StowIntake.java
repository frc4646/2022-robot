package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.intake.IntakeExtend;

public class StowIntake extends SequentialCommandGroup {
  public StowIntake() {
    addCommands(
      new IntakeExtend(false),
      new AgitateOpenLoop(0.5),
      new WaitCommand(0.5),
      new AgitateOpenLoop(0.0),
      new WaitCommand(0.5),
      new AgitateOpenLoop(1.0),
      new WaitCommand(0.5),
      new AgitateOpenLoop(0.0),
      new WaitCommand(0.5)
    );
  }
}
