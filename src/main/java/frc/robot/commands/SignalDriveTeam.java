package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class SignalDriveTeam extends CommandBase {
  private final Diagnostics diagnostics = RobotContainer.DIAGNOSTICS;
  private final Climber climber = RobotContainer.CLIMBER;
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final Turret turret = RobotContainer.TURRET;

  public SignalDriveTeam() {
    addRequirements(diagnostics);
  }

  @Override
  public void execute() {
    if (isTurretFaultPresent()) {
      diagnostics.setState(Constants.Diagnostic.FAULT_TURRET);
    } else if (isWrongAllianceCargoPresent()) {
      diagnostics.setState(Constants.Diagnostic.FAULT_CARGO);
    } else if (isClimbing()) {
      diagnostics.setState(Constants.Diagnostic.CLIMBING);
    } else if (isShooting()) {
      diagnostics.setState(Constants.Diagnostic.SHOOTING);
    } else if (isTurretAimed()) {
      diagnostics.setState(Constants.Diagnostic.TURRET_AIMED);
    } else {
      diagnostics.setStateOkay();
    }
  }

  private boolean isTurretAimed() { return false; }  // TODO
  private boolean isWrongAllianceCargoPresent() { return false; }  // TODO
  private boolean isClimbing() { return false; }  // TODO
  private boolean isShooting() { return shooter.isShooting(); }
  private boolean isTurretFaultPresent() { return false; }  // TODO return !turret.hasBeenZeroed();

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
