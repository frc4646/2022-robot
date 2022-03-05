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
  private final OperatorControls controls = RobotContainer.CONTROLS.getOperator();

  public TurretAim() {
    addRequirements(turret);
  }

  @Override
  public void execute() {
    double setpoint = turret.getPosition();
    double feedforward = 0.0;
    double stick = controls.getTurretStick();
    int snap = controls.getTurretSnap();

    if (isVisionWanted()) {
      setpoint -= vision.getDegreesX();
    } else if (snap != -1) {
      setpoint = snap;
    } else if (Math.abs(stick) >= Constants.TURRET.STICK_DEADBAND) {
      setpoint += stick * Constants.TURRET.STICK_GAIN;
    }
    turret.setSetpointPositionPID(setpoint, feedforward);
  }

  private boolean isVisionWanted() {
    // TODO deadzones where climber is
    return vision.isTargetPresent() && !controls.getFn();
  }
}
