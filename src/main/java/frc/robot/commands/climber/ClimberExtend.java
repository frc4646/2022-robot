package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberExtend extends InstantCommand {
  private final Climber subsystem = RobotContainer.CLIMBER;
  private final boolean extend;

  public ClimberExtend(boolean extend) {
    this.extend = extend;
  }

  @Override
  public void initialize() {
    subsystem.setArms(extend);
  }
}
