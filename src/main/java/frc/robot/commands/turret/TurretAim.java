package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretAim extends CommandBase {
  private final double GAIN_STABILITY = 0.95;
  private final double GAIN_YAW_RATE = 0.0;  // TODO test non-zero
  private final Turret turret = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final OperatorControls operator = RobotContainer.CONTROLS.getOperator();

  public TurretAim() {
    addRequirements(turret);
  }

  @Override
  public void execute() {
    double stick = operator.getTurretStick();
    int snap = operator.getTurretSnap();
    double setpoint = turret.getPosition();
    double feedforward = 0.0;

    if (vision.isTargetPresent() && !operator.getFn()) {
      setpoint += vision.getTurretError() * GAIN_STABILITY;
      feedforward -= drive.getHeadingRate() * GAIN_YAW_RATE;
    } else if (snap != -1) {
      setpoint = snap;
    } else if (Math.abs(stick) >= Constants.TURRET.STICK_DEADBAND) {
      setpoint += stick * Constants.TURRET.STICK_GAIN;
    }
    turret.setSetpointMotionMagic(setpoint, feedforward);
  }
}
