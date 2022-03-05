package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretAim extends CommandBase {
  private final Turret turret = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;
  private final OperatorControls operator = RobotContainer.CONTROLS.getOperator();

  public TurretAim() {
    addRequirements(turret);
  }

  @Override
  public void execute() {
    double position = turret.getPosition();
    double setpoint = position;
    double feedforward = 0.0;
    double stick = operator.getTurretStick();
    int snap = operator.getTurretSnap();

    if (isVisionWanted(position)) {
      setpoint -= vision.getDegreesX();
    } else if (snap != -1) {
      setpoint = snap;
    } else if (Math.abs(stick) >= Constants.TURRET.STICK_DEADBAND) {
      setpoint += stick * Constants.TURRET.STICK_GAIN;
      // turret.setOpenLoop(stick);
    }
    turret.setSetpointMotionMagic(setpoint, feedforward);
  }

  private boolean isVisionWanted(double position) {
    return vision.isTargetPresent() && !operator.getFn() && !isInDeadzone(position);
  }

  private boolean isInDeadzone(double position) {
    boolean inDeadzoneR = position < 150.0 && position > 45.0;
    boolean inDeadzoneL = position > 210.0 && position < 315.0;
    return inDeadzoneR || inDeadzoneL;
  }
}
