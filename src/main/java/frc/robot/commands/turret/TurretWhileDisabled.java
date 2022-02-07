package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretWhileDisabled extends CommandBase {
  private Turret subsystem = RobotContainer.TURRET;

  public TurretWhileDisabled() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    subsystem.resetIfAtHome();
    if (!subsystem.hasBeenZeroed()) {
        // mLED.setTurretFault();
    } else {
        // mLED.clearTurretFault();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
