package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitatorOpenLoop;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.wait.WaitForShooterVelocity;
import frc.robot.util.ShootSetpoint;

public class ShootOpenLoop extends SequentialCommandGroup {
  public ShootOpenLoop() {
    addCommands(
      deadline(        
        new WaitForShooterVelocity().withTimeout(Constants.SHOOTER.OPEN_LOOP_REV_SECONDS),
        new ShooterVelocity(ShootSetpoint.DEFAULT),
        new AgitatorOpenLoop(Constants.AGITATOR.OPEN_LOOP_SHOOT)
      ),
      new FireCargo()
    );
  }
}
