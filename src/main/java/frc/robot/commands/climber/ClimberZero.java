package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberZero extends CommandBase {
  private final Climber subsystem;

  public ClimberZero() {
    subsystem = RobotContainer.CLIMBER;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setSoftLimitsEnabled(false);
    subsystem.setOpenLoop(-Constants.Climber.OPEN_LOOP_ZERO);
  }

  @Override
  public boolean isFinished() {
    return subsystem.atHomingLocation(true) && subsystem.atHomingLocation(false);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.zeroSensors(true);
    subsystem.zeroSensors(false);
    subsystem.setSoftLimitsEnabled(true);
    subsystem.setOpenLoop(0.0);
  }
}
