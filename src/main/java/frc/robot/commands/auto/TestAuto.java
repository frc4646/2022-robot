package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.vision.VisionLED;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto() {
    addCommands(
      new VisionLED(true),
      new WaitCommand(2.0),
      new VisionLED(false));
  }
}
