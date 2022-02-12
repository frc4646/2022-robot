package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LEDMode;

public class VisionLED extends CommandBase {
  private final Vision subsystem = RobotContainer.VISION;
  private final LEDMode mode;

  public VisionLED(LEDMode mode) {
    addRequirements(subsystem);
    this.mode = mode;
  }

  @Override
  public void initialize() {
    subsystem.setLED(mode);
  }
}
