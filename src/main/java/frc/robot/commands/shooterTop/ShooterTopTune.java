package frc.robot.commands.shooterTop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterTop;

public class ShooterTopTune extends CommandBase {
  public static final String DASHBOARD_KEY_SHOOTER_TUNE = "Tune: ShootTop RPM";
  
  private ShooterTop subsystem = RobotContainer.SHOOTER_TOP;

  public ShooterTopTune() {
    addRequirements(subsystem);
    SmartDashboard.putNumber(DASHBOARD_KEY_SHOOTER_TUNE, 0.0);
  }

  @Override
  public void initialize() {
    double setpoint = SmartDashboard.getNumber(DASHBOARD_KEY_SHOOTER_TUNE, 0.0);
    subsystem.setClosedLoop(setpoint);
  }
}
