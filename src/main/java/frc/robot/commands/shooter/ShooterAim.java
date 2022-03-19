package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Vision;
import frc.robot.util.ShootSetpoint;

public class ShooterAim extends CommandBase {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final Vision vision = RobotContainer.VISION;
  private final OperatorControls operator = RobotContainer.CONTROLS.getOperator();

  public ShooterAim() {
    addRequirements(shooter, shooterTop);
  }

  @Override
  public void execute() {
    double stickTrim = operator.getShooterTrim();
    double trim = Math.abs(stickTrim) >= Constants.SHOOTER.DEADBAND ? stickTrim * Constants.SHOOTER.RPM_TRIM : 0.0;
    boolean override = operator.getFn();
    ShootSetpoint setpoint = vision.isTargetPresent() && !override ? vision.getShooterRPM() : ShootSetpoint.DEFAULT;
    setpoint = new ShootSetpoint(setpoint.rpmBottom + trim, setpoint.rpmTop + trim);

    shooter.setClosedLoop(setpoint, true);
    shooterTop.setClosedLoop(setpoint);
  }
}
