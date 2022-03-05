package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberZero extends CommandBase {
  private final Climber subsystem = RobotContainer.CLIMBER;

  public ClimberZero() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setSoftLimitsEnabled(false);
    subsystem.setOpenLoop(-Constants.CLIMBER.OPEN_LOOP_ZERO);
  }

  @Override
  public boolean isFinished() {
    boolean bothAtHome = subsystem.isAtHomingLocation(true) && subsystem.isAtHomingLocation(false); 
    return subsystem.isZeroed() || bothAtHome;
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.zeroSensors(true);
    subsystem.zeroSensors(false);
    subsystem.setSoftLimitsEnabled(true);
    subsystem.setOpenLoop(0.0);
  }
}
