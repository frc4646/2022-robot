package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;

public class Hoodextend extends InstantCommand{
  private final Hood subsystem = RobotContainer.HOOD;
  private final boolean extend;
  public Hoodextend(boolean extend) {
    addRequirements(subsystem);
    this.extend = extend;
  }
  @Override
  public void initialize() {
    subsystem.setExtend(extend);
  }
}
