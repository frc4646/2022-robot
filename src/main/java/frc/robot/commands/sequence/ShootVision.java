package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitatorPulse;
import frc.robot.commands.feeder.FeederLoadCargo;
import frc.robot.commands.shooter.ShooterAim;
import frc.robot.commands.shooter.ShooterLockRPM;
import frc.robot.commands.turret.TurretLockPosition;
import frc.robot.commands.wait.WaitForAim;

public class ShootVision extends SequentialCommandGroup {
  public ShootVision() {
    addCommands(
      deadline(
        parallel(
          new WaitForAim(),
          new FeederLoadCargo()
        ),
        new ShooterAim(),
        new AgitatorPulse(Constants.AGITATOR.OPEN_LOOP_LOAD * 1.5, 0.5)
      ),
      parallel(
        new ShooterLockRPM(),  // Protect form obstructions, ex: first cargo in flight
        new TurretLockPosition()
      ),
      new FireCargo()
    );
  }
}
