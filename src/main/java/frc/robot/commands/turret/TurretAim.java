package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.team254.CardinalDirection;
import frc.team254.util.Util;

public class TurretAim extends CommandBase {
  private final Turret turret = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;
  private final OperatorControls controls = RobotContainer.CONTROLS.operator;

  public TurretAim() {
    addRequirements(turret);
  }

  @Override
  public void execute() {
    double setpoint = turret.getPosition();
    double feedforward = 0.0;  
    double stick = controls.getTurretStick();
    CardinalDirection snap = controls.getTurretSnap();

    if (vision.isTargetPresent()) {
      setpoint -= vision.getDegreesX();
    }
    // else if (snap != CardinalDirection.NONE) {
    //   setpoint = snap.getRotation().getDegrees();
    // }
    else if (stick >= 0.0) {
      setpoint += stick * Constants.Turret.OPEN_LOOP_GAIN;
    }
    turret.setSetpointPositionPID(setpoint, feedforward);
  }
}
