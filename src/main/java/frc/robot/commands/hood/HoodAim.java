package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Vision;

public class HoodAim extends CommandBase {
  private final Hood hood = RobotContainer.HOOD;
  private final Vision vision = RobotContainer.VISION;
  
  public HoodAim() {
    addRequirements(hood);
  }

  @Override
  public void execute() {
    double setpoint = 0.0; // TODO hood.getPosition();
    double feedforward = 0.0;

    if (vision.isTargetPresent()) {
      // setpoint = vision.getHoodDegrees();
    }
    // TODO hood.setSetpointPositionPID(setpoint, feedforward);
  }
}
