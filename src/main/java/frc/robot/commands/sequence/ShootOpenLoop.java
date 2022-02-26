package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.shooter.ShooterOpenLoop;

public class ShootOpenLoop extends SequentialCommandGroup {
  public ShootOpenLoop() {
    addCommands(
      new ShooterOpenLoop(Constants.Shooter.OPEN_LOOP),
      new ParallelCommandGroup(new IntakeActivate(0), new AgitateOpenLoop(0.45)),
      new WaitCommand(Constants.Shooter.OPEN_LOOP_REV_SECONDS),   // TODO make closed loop at default RPM, wait for RPM, then feed
      new FeederOpenLoop(0.5)
    );
  }
}
