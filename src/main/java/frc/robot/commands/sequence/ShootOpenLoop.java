package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.shooter.ShooterOpenLoop;

public class ShootOpenLoop extends SequentialCommandGroup {
  public ShootOpenLoop() {
    addCommands(
      new ShooterOpenLoop(Constants.Shooter.OPEN_LOOP),
      new AgitateOpenLoop(Constants.Agitator.OPEN_LOOP_SHOOTING),
      new WaitCommand(Constants.Shooter.OPEN_LOOP_REV_SECONDS),   // TODO make closed loop at default RPM, wait for RPM, then feed
      new FeederOpenLoop(Constants.Feeder.OPEN_LOOP_SHOOTING)
    );
  }
}
