package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretAim extends CommandBase {
  private final Turret turret = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;
  private final RobotState state = RobotContainer.ROBOT_STATE;
  private final OperatorControls operator = RobotContainer.CONTROLS.getOperator();

  public TurretAim() {
    addRequirements(turret);
  }

  @Override
  public void execute() {
    double stick = operator.getTurretStick();
    int snap = operator.getTurretSnap();
    double setpoint = turret.getPosition();

    if (vision.isTargetPresent() && !operator.getFn()) {
      setpoint = state.getTurretVisionSetpoint();
    } else if (snap != -1) {
      setpoint = snap;
    } else if (Math.abs(stick) >= Constants.TURRET.STICK_DEADBAND) {
      setpoint += stick * Constants.TURRET.STICK_GAIN;
    }
    turret.setSetpointMotionMagic(setpoint, 0.0);
  }
}
