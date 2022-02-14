package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.ShootOpenLoop;

public class TwoCargoAuto extends SequentialCommandGroup {
  public TwoCargoAuto() {
    addCommands(
      new ShootOpenLoop(),
      new IntakeExtend(true),
      new IntakeActivate(Constants.Intake.OPEN_LOOP),
      new DriveOpenLoop(-.1),
      new WaitCommand(1.0),
      new DriveOpenLoop(0.0),
      new WaitCommand(1.0),
      new ShootOpenLoop()
    );
  }
}
