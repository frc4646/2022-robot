package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveTrajectory;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto() {
    addCommands(
      new DriveTrajectory(null)  // TODO
    );
  }
}
