package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class WaitForVisionTarget extends WaitUntilCommand {
  public WaitForVisionTarget() {
    super(RobotContainer.VISION::isTargetPresent);
  }
}
