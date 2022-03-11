package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederLoadCargo;
import frc.robot.commands.shooter.ShooterAim;

public class ShootVision extends SequentialCommandGroup {
  public ShootVision() {
    addCommands(
      deadline(
        parallel(
          new WaitForAim(),
          new FeederLoadCargo()
        ),
        new ShooterAim(),
        new AgitateOpenLoop(Constants.AGITATOR.OPEN_LOOP_SHOOT)
        // TODO should drivetrain lock?
      ),
      new FireCargo()
    );
  }
}
