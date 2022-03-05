package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShooterAim extends CommandBase {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final Vision vision = RobotContainer.VISION;
  private final OperatorControls operator = RobotContainer.CONTROLS.getOperator();

  public ShooterAim() {
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    double setpoint = Constants.SHOOTER.RPM_DEFAULT;
    double trim = operator.getShooterTrim();
    boolean override = operator.getFn();

    if (vision.isTargetPresent() && !override) {
      setpoint = vision.getShooterRPM();
    }
    shooter.setClosedLoop(setpoint + trim);
  }
}
