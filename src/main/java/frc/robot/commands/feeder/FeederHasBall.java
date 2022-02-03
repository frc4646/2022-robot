package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederHasBall extends CommandBase {
  private final Feeder subsystem = RobotContainer.FEEDER;

  public FeederHasBall() {
    // Do not require subsystem so the other command isn't interrupted
  }

  @Override
  public boolean isFinished() {
    return subsystem.isBallPresent();
  }
}
