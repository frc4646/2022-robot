package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.sequence.StowIntake;

public class TwoCargoAuto extends SequentialCommandGroup {
  public TwoCargoAuto() {
    addCommands(
      new ShootOpenLoop(),
      new DeployIntake(),
      new DriveOpenLoop(-.1),
      new WaitCommand(1.0),
      new DriveOpenLoop(0.0),
      new StowIntake(),
      new ShootOpenLoop()
    );
  }
}
