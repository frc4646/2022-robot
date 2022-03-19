package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ModeRight extends ModeBase {
  public ModeRight(STRATEGY_PHASE_2 strategy) {
    addCommands(
      new ResetAuto(Paths.RIGHT.CARGO_2.getInitialPose()),
      new GrabCargo2ThenShoot(Paths.RIGHT.CARGO_2, 115.0),
      selectPhase2(strategy)
    );
  }

  private Command selectPhase2(STRATEGY_PHASE_2 strategy) {
    Command phase2 = new WaitCommand(15.0);

    return phase2;
  } 
}
