package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class WaitForFeederEmpty extends CommandBase {
  private final Feeder subsystem = RobotContainer.FEEDER;
  
  @Override
  public boolean isFinished() {
    return !subsystem.isCargoPresent();  // TODO
  }
}
