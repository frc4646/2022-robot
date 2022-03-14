package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterTune extends CommandBase {
  public static final String DASHBOARD_KEY_SHOOTER_TUNE = "Tune: Shoot RPM";
  
  private Shooter subsystem = RobotContainer.SHOOTER;

  public ShooterTune() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    double setpoint = SmartDashboard.getNumber(DASHBOARD_KEY_SHOOTER_TUNE, 0.0);
    subsystem.setClosedLoop(setpoint);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isStable();
  }
}
