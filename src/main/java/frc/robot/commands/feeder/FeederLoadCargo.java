package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederLoadCargo extends CommandBase {
  private Feeder subsystem = RobotContainer.FEEDER;
  private final double output;

  public FeederLoadCargo(double percent) {
    addRequirements(subsystem);
    output = percent;
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(output);
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
