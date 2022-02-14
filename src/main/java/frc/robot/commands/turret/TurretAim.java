package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretAim extends CommandBase {
  private final Turret subsystem = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;
  private final OperatorControls controls = RobotContainer.CONTROLS.operator;

  public TurretAim() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double setpoint = subsystem.getPosition();
    double feedforward = 0.0;
    double stick = controls.getTurretStick();

    if (vision.isTargetPresent()) {
      // TODO ask vision what angle to use
    }
    else if (stick >= 0.0) {
      setpoint += stick * Constants.Turret.OPEN_LOOP_GAIN;
    }
    subsystem.setSetpointPositionPID(setpoint, feedforward);
  }
}
