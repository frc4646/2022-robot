package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;

public class ShooterTune extends CommandBase {
  public static final String DASHBOARD_KEY_SHOOTER_TUNE = "Tune: Shoot RPM";
  public static final String DASHBOARD_KEY_SHOOTER_TOP_TUNE = "Tune: ShootTop RPM";
  
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;

  public ShooterTune() {
    addRequirements(shooter, shooterTop);
    SmartDashboard.putNumber(DASHBOARD_KEY_SHOOTER_TUNE, 0.0);
    SmartDashboard.putNumber(DASHBOARD_KEY_SHOOTER_TOP_TUNE, 0.0);
  }

  @Override
  public void initialize() {
    shooter.setClosedLoop(SmartDashboard.getNumber(DASHBOARD_KEY_SHOOTER_TUNE, 0.0));
    shooterTop.setClosedLoop(SmartDashboard.getNumber(DASHBOARD_KEY_SHOOTER_TOP_TUNE, 0.0));
  }
}
