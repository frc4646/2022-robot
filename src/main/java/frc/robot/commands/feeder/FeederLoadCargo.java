package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederLoadCargo extends CommandBase {
  private Feeder subsystem = RobotContainer.FEEDER;

  public FeederLoadCargo() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(Constants.Feeder.OPEN_LOOP_LOADING);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isCargoPresent();
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      subsystem.setOpenLoop(0.0);  // Don't stop feeder when shooting second cargo
    }
  }
}
