package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeederLoadCargo;
import frc.robot.commands.feeder.FeederOpenLoop;

public class FireCargo extends SequentialCommandGroup {
  public FireCargo() {
    addCommands(
      new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_SHOOT),
      new WaitCommand(0.25),
      new FeederLoadCargo().withTimeout(0.5),
      // new WaitUntilCommand(RobotContainer.SHOOTER::isStable),  // TODO try this
      new WaitCommand(0.25),
      new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_SHOOT),
      new WaitCommand(0.25)
    );
  }
}
