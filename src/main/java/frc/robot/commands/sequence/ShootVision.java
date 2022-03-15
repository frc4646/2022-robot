package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederLoadCargo;
import frc.robot.commands.shooter.ShooterAim;
import frc.robot.commands.shooter.ShooterLockRPM;
import frc.robot.commands.shooter.ShooterTopAim;
import frc.robot.commands.shooter.ShooterTopLockRPM;

public class ShootVision extends SequentialCommandGroup {
  public ShootVision() {
    addCommands(
      deadline(
        parallel(
          new WaitForAim(),
          new FeederLoadCargo()
        ),
        parallel(
          new ShooterAim(),
          new ShooterTopAim()
        ),
        new AgitateOpenLoop(Constants.AGITATOR.OPEN_LOOP_SHOOT)
      ),
      parallel(
        new ShooterLockRPM(),  // Protect form obstructions, ex: first cargo in flight
        new ShooterTopLockRPM()
      ),
      new FireCargo()
    );
  }
}
