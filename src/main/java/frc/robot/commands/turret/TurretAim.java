package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretAim extends CommandBase {
  private final Turret subsystem = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;
  private final OperatorControls controls = RobotContainer.CONTROLS.operator;
  private final double GAIN_JOG = 4.0;  // TODO tune

  public TurretAim() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double setpoint = subsystem.getPosition();
    double feedforward = 0.0;

    if (vision.isTargetPresent()) {
      // TODO ask vision what angle to use
    }
    else if (controls.getTurretJog() >= 0.0) {
      setpoint += controls.getTurretJog() * GAIN_JOG;
    }
    subsystem.setSetpointPositionPID(setpoint, feedforward);
  }
}
