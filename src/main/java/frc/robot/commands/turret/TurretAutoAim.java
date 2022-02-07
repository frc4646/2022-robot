package frc.robot.commands.turret;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretAutoAim extends CommandBase {  
  private final Turret subsystem = RobotContainer.TURRET;

  private Optional<Rotation2d> hint;

  public TurretAutoAim() {
    addRequirements(subsystem);
  }  
  
  public TurretAutoAim(Rotation2d hint) {
    this();
    this.hint = Optional.of(hint);
  }

  @Override
  public void initialize() {
    if (hint.isPresent()) {
      // TODO
    }
    // TODO
  }
}
