package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.shooter.ShooterAim;

public class ShootVision extends SequentialCommandGroup {
  public ShootVision() {
    addCommands(
      deadline(
        new WaitForAim(),
        new ShooterAim(),
        new IntakeActivate(0.5),
        new AgitateOpenLoop(0.45)
        // TODO should drivetrain lock?
      ),
      new FeederOpenLoop(1.0)
    );
  }
}
