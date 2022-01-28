package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.ShootOpenLoop;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto() {
    addCommands(
     // new ShootOpenLoop(),
     //new IntakeExtend(true),
     // new IntakeActivate(.5),
     // new DriveOpenLoop(.3).withTimeout(1.0),
     // new DriveOpenLoop(.15).withTimeout(0.5),
     // new DriveOpenLoop(0.0),
     // new ShootOpenLoop()
     new ShootOpenLoop(),
    new DriveOpenLoop(.1),
    new WaitCommand(1),
   new DriveOpenLoop(0)
    );
  }
}
