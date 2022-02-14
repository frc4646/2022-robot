package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretZero extends CommandBase {
  private final Turret subsystem = RobotContainer.TURRET;

  public TurretZero() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // TODO
  }

  @Override
  public boolean isFinished() {
    return subsystem.hasBeenZeroed();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setOpenLoop(0.0);
    subsystem.zeroSensors();
  }
}
