package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretPosition extends CommandBase {
  private final Turret subsystem = RobotContainer.TURRET;
  private final double setpoint, tolerance;

  public TurretPosition(double angle, double tolerance) {
    addRequirements(subsystem);
    setpoint = angle;
    this.tolerance = tolerance;
  }

  @Override
  public void initialize() {
    //subsystem.setSetpointPositionPID(setpoint, 0.0);
    subsystem.setSetpointMotionMagic(setpoint, 0.0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(subsystem.getPosition() - setpoint) < tolerance;  // TODO use isOnTarget
  }
}
