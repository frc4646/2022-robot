package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.shooter.ShooterOpenLoop;

public class ShootOpenLoop extends SequentialCommandGroup {
  public ShootOpenLoop(double percentShooter) {
    addCommands(
      new ShooterOpenLoop(percentShooter),
      new WaitCommand(Constants.Shooter.REV_TIME),
      new FeederOpenLoop(Constants.Feeder.PERCENT_OPEN_LOOP)
    );
  }
}
