package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederLoadCargo extends CommandBase {
  private final Feeder subsystem = RobotContainer.FEEDER;

  public FeederLoadCargo() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(Constants.FEEDER.OPEN_LOOP_LOAD);
  }
  
  @Override
  public boolean isFinished() {
    return subsystem.isShooterLoaded();  // TODO refactor
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setOpenLoop(0.0);
  }
}
