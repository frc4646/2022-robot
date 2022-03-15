package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.shooter.WaitForShooterVelocity;
import frc.robot.commands.shooterTop.ShooterTopVelocity;
import frc.robot.commands.shooterTop.WaitForShooterTopVelocity;

public class ShootOpenLoop extends SequentialCommandGroup {
  public ShootOpenLoop() {
    addCommands(
      deadline(
        parallel(
          new WaitForShooterVelocity().withTimeout(Constants.SHOOTER.OPEN_LOOP_REV_SECONDS),
          new WaitForShooterTopVelocity().withTimeout(Constants.SHOOTER.OPEN_LOOP_REV_SECONDS)
        ),
        new ShooterVelocity(Constants.SHOOTER.RPM_DEFAULT),
        new ShooterTopVelocity(Constants.SHOOTER_TOP.RPM_DEFAULT),
        new AgitateOpenLoop(Constants.AGITATOR.OPEN_LOOP_SHOOT)
      ),
      new FireCargo()
    );
  }
}
