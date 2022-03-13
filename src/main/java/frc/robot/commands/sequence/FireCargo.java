package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.feeder.FeederLoadCargo;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.feeder.WaitForFeederState;

public class FireCargo extends SequentialCommandGroup {
  public FireCargo() {
    addCommands(
      new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_SHOOT),
      new WaitCommand(0.25),
      //new WaitForFeederState(false),  // TODO try this
      //new WaitForFeederState(true),  // TODO try this
      new FeederLoadCargo().withTimeout(0.5),
      //new FeederOpenLoop(0.0),  // pulse
      // new WaitUntilCommand(RobotContainer.SHOOTER::isStable),  // TODO try this
      new WaitCommand(0.25),
      new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_SHOOT),
      new WaitCommand(0.25)
    );
  }
}
