package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberTune extends CommandBase {
  public static final String DASHBOARD_KEY_TUNE = "Tune: Climber PercentUp";
  
  private final Climber subsystem = RobotContainer.CLIMBER;

  public ClimberTune() {
    addRequirements(subsystem);
    SmartDashboard.putNumber(DASHBOARD_KEY_TUNE, 0.0);
  }

  @Override
  public void initialize() {
    double setpoint = SmartDashboard.getNumber(DASHBOARD_KEY_TUNE, 0.0);
    subsystem.setClosedLoopPosition(setpoint);
  }
}
