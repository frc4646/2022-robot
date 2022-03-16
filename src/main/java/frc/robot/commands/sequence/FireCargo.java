package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeederLoadCargo;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.shooter.WaitForShooterVelocity;

public class FireCargo extends SequentialCommandGroup {
  public FireCargo() {
    addCommands(
      new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_SHOOT),
      new WaitCommand(0.25),
      new FeederLoadCargo().withTimeout(1.0),
      new WaitForShooterVelocity(),
      new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_SHOOT),
      new WaitCommand(0.5)
    );
  }
}
