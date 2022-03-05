package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;

public class ExhaustIntake extends SequentialCommandGroup {
  public ExhaustIntake() {
    addCommands(
      new IntakeExtend(true),
      new IntakeActivate(-Constants.INTAKE.OPEN_LOOP),
      new WaitCommand(0.5),  // Wait for intake out and running before feeder pulse
      parallel(
        new AgitateOpenLoop(-Constants.AGITATOR.OPEN_LOOP_EXHAUST),
        new FeederOpenLoop(-Constants.FEEDER.OPEN_LOOP_EXHAUST)
      ),
      new WaitCommand(Constants.FEEDER.TIMEOUT_EXHAUST),
      new FeederOpenLoop(0.0),
      // TODO wait for correct cargo loading sensor so reload correct cargo + no need to stop feeder exhaust?
      new WaitCommand(2.0)
    );
  }
}
