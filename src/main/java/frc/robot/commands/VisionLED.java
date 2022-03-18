package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LEDMode;

public class VisionLED extends InstantCommand {
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
