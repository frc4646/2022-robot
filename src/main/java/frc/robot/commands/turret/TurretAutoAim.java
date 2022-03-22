package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretAutoAim extends SequentialCommandGroup {
  private final Turret turret = RobotContainer.TURRET;

  public TurretAutoAim() {
    addRequirements(turret);
    addCommands(new SelectCommand(TurretAutoAim::select));
  }

  public static Command select() {
    // TODO vision owl
    return new TurretAim();
  }
}
