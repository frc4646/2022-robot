package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberArms extends InstantCommand {
  final Climber subsystem = RobotContainer.CLIMBER;
  final boolean extend;

  public ClimberArms(boolean extend) {
    this.extend = extend;
  }

  @Override
  public void initialize() {
    subsystem.setArms(extend);
  }
}
