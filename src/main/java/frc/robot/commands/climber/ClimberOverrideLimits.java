package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberOverrideLimits extends CommandBase {
  private final Climber subsystem = RobotContainer.CLIMBER;

  @Override
  public void initialize() {
    subsystem.setSoftLimitsEnabled(false);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setSoftLimitsEnabled(true);
  }
}
