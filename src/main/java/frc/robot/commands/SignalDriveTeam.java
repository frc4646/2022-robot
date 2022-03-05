package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class SignalDriveTeam extends CommandBase {
  private final Diagnostics diagnostics = RobotContainer.DIAGNOSTICS;
  private final Climber climber = RobotContainer.CLIMBER;
  private final ColorSensor colorSensor = RobotContainer.COLOR_SENSOR;
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final Turret turret = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;

  public SignalDriveTeam() {
    addRequirements(diagnostics);
  }

  @Override
  public void execute() {
    if (isTurretFaultPresent()) {
      diagnostics.setState(Constants.DIAGNOSTICS.FAULT_TURRET);
    } else if (isWrongAllianceCargoPresent()) {
      diagnostics.setState(Constants.DIAGNOSTICS.FAULT_CARGO);
    } else if (isClimbing()) {
      diagnostics.setState(Constants.DIAGNOSTICS.CLIMBING);
    } else if (isShooting()) {
      diagnostics.setState(Constants.DIAGNOSTICS.SHOOTING);
    } else if (isTurretAimed()) {
      diagnostics.setState(Constants.DIAGNOSTICS.TURRET_AIMED);
    } else {
      diagnostics.setStateOkay();
    }
  }

  private boolean isTurretAimed() { return false; }  // TODO vision.isTargetPresent() && turret.isOnTarget()
  private boolean isWrongAllianceCargoPresent() { return colorSensor.isWrongCargo(); }
  private boolean isClimbing() { return climber.isInClimbMode(); }
  private boolean isShooting() { return shooter.isShooting(); }
  private boolean isTurretFaultPresent() { return !turret.hasBeenZeroed(); }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
