package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LEDMode;

public class VisionLED extends InstantCommand {
  private final Vision subsystem = RobotContainer.VISION;

  private final boolean output;

  public VisionLED(boolean enable) {
    addRequirements(subsystem);
    output = enable;
  }

  @Override
  public void initialize() {
    LEDMode mode = (output) ? LEDMode.ON : LEDMode.OFF;
    subsystem.setLED(mode);
  }
}
