package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class SignalDriveTeam extends CommandBase {
  private final Diagnostics diagnostics = RobotContainer.DIAGNOSTICS;
  private final Climber climber = RobotContainer.CLIMBER;
  private final Feeder feeder = RobotContainer.FEEDER;
  private final Turret turret = RobotContainer.TURRET;
  private final Vision vision = RobotContainer.VISION;
  private final RobotState state = RobotContainer.ROBOT_STATE;

  public SignalDriveTeam() {
    addRequirements(diagnostics);
  }

  @Override
  public void execute() {
    if (isTurretFaultPresent()) {
      diagnostics.setState(Constants.DIAGNOSTICS.FAULT_TURRET);
    // } else if (isClimberFaultPresent()) {
    //   diagnostics.setState(Constants.DIAGNOSTICS.FAULT_CLIMBER);
    // } else if (isClimbing()) {
    //   diagnostics.setState(Constants.DIAGNOSTICS.CLIMBING);
    } else if (canPressShoot()) {
      diagnostics.setState(Constants.DIAGNOSTICS.CAN_PRESS_SHOOT);
    } else if (isCargoLoaded()) {
      diagnostics.setState(Constants.DIAGNOSTICS.CARGO_LOADED);
    } else {
      diagnostics.setStateOkay();
    }
  }

  private boolean canPressShoot() { return vision.isTargetPresent() && vision.isInShootRange() && state.isTwoCargoLoaded(); }
  private boolean isCargoLoaded() { return feeder.isCargoIndexed(); }
  private boolean isClimberFaultPresent() { return !climber.isZeroed(); }
  private boolean isClimbing() { return climber.isInClimbMode(); }
  private boolean isTurretFaultPresent() { return !turret.hasBeenZeroed(); }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
