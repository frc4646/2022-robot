package frc.robot.commands.shooterTop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Vision;

public class ShooterTopAim extends CommandBase {
  private final ShooterTop shooter = RobotContainer.SHOOTER_TOP;
  private final Vision vision = RobotContainer.VISION;
  private final OperatorControls operator = RobotContainer.CONTROLS.getOperator();

  public ShooterTopAim() {
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    double setpoint = Constants.SHOOTER.RPM_DEFAULT;
    double stickTrim = operator.getShooterTrim();
    double trim = Math.abs(stickTrim) >= Constants.SHOOTER.DEADBAND ? stickTrim * Constants.SHOOTER_TOP.RPM_TRIM : 0.0;
    boolean override = operator.getFn();

    if (vision.isTargetPresent() && !override) {
      setpoint = vision.getShooterRPMTop();
    }
    shooter.setClosedLoop(setpoint + trim);
  }
}
