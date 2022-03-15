package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ModeMiddle extends ModeBase {
  public ModeMiddle(STRATEGY_PHASE_2 strategy) {
    addCommands(
      new GrabCargo2ThenShoot(Paths.MIDDLE.CARGO_2, 115.0),
      selectPhase2(strategy)
    );
  }

  private Command selectPhase2(STRATEGY_PHASE_2 strategy) {
    Command phase2 = new WaitCommand(15.0);

    if (strategy == STRATEGY_PHASE_2.HUMAN_PLAYER)
      phase2 = new HumanPlayerThenShoot(Paths.MIDDLE.pathHumanPlayer, Paths.MIDDLE.pathShoot3And4);

    return phase2;
  }
}