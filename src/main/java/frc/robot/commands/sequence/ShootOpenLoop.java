package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.shooter.WaitForShooterVelocity;

public class ShootOpenLoop extends SequentialCommandGroup {
  public ShootOpenLoop() {
    addCommands(
      new ShooterVelocity(Constants.Shooter.RPM_DEFAULT),
      new AgitateOpenLoop(Constants.Agitator.OPEN_LOOP_SHOOT),
      new WaitForShooterVelocity().withTimeout(Constants.Shooter.OPEN_LOOP_REV_SECONDS),
      new FireCargo()
    );
  }
}
