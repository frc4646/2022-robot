package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class HoodAutoZero extends CommandBase {
  private final Hood subsystem = RobotContainer.HOOD;

  public HoodAutoZero() {
    addRequirements(subsystem);
  }

  // TODO
}
