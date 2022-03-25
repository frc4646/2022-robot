package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitatorPulse;
import frc.robot.commands.feeder.FeederLoadCargo;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.intake.IntakeOpenLoop;

public class StowIntake extends SequentialCommandGroup {
  public StowIntake() {
    this(0.5);
  }

  public StowIntake(double delaySolenoids) {
    addCommands(
      new WaitCommand(delaySolenoids),
      new IntakeExtend(false),
      deadline(
        new WaitCommand(Constants.INTAKE.TIMEOUT_STOW),
        // new FeederLoadCargo(),
        // new AgitatorPulse(Constants.AGITATOR.OPEN_LOOP_LOAD * 1.0, 0.5),
        new IntakeOpenLoop(0.0)
      )
    );
  }
}
