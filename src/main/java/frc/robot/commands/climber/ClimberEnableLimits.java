package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberEnableLimits extends InstantCommand {
  private final Climber subsystem = RobotContainer.CLIMBER;
  private final boolean enable;

  public ClimberEnableLimits(boolean enable) {
    this.enable = enable;
  }

  @Override
  public void initialize() {
    subsystem.setSoftLimitsEnabled(enable);
  }
}
