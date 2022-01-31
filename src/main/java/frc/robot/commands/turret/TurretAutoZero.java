package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretAutoZero extends CommandBase {
  private final Turret subsystem = RobotContainer.TURRET;

  public TurretAutoZero() {
    addRequirements(subsystem);
  }
}
