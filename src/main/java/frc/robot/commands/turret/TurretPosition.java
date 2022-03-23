package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretPosition extends CommandBase {
  private final Turret subsystem = RobotContainer.TURRET;
  private final double setpoint;

  public TurretPosition(double angle) {
    addRequirements(subsystem);
    setpoint = angle;
  }

  @Override
  public void initialize() {
    subsystem.setSetpointMotionMagic(setpoint, 0.0);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isOnTarget();
  }
}
