package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederHasBall extends CommandBase {
  private final Feeder subsystem = RobotContainer.FEEDER;

  public FeederHasBall() {
    addRequirements(subsystem);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isBallPresent();
  }
}
