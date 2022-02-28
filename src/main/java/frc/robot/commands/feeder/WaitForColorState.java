package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.ColorSensor.STATE;

public class WaitForColorState extends CommandBase {
  private final ColorSensor subsystem = RobotContainer.COLOR_SENSOR;
  private final STATE state;

  public WaitForColorState(STATE state) {
    this.state = state;
  }
  
  @Override
  public boolean isFinished() {
    return subsystem.getState() == state;
  }
}
