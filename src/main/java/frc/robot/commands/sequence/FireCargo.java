package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeederOpenLoop;

public class FireCargo extends SequentialCommandGroup {
  public FireCargo() {
    addCommands(
      new FeederOpenLoop(Constants.Feeder.OPEN_LOOP_SHOOT),
      new WaitCommand(0.5),  // TODO switch to wait command + delay for consistency
      new FeederOpenLoop(0.0),  // pulse
      new WaitCommand(0.5),  // TODO switch to wait command + delay for consistency
      new FeederOpenLoop(Constants.Feeder.OPEN_LOOP_SHOOT),
      new WaitCommand(0.5)  // TODO switch to wait command + delay for consistency
    );
  }
}
