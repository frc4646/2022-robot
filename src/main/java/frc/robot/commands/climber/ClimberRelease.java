package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberRelease extends CommandBase {
  private final Climber subsystem = RobotContainer.CLIMBER;

  public ClimberRelease() {
    addRequirements(subsystem);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setBrakeMode(false);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
