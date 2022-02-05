package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberArms extends CommandBase {
  final Climber subsystem = RobotContainer.CLIMBER;

  final boolean extend;

  public ClimberArms(boolean extend) {
    addRequirements(subsystem);
    this.extend = extend;
  }

  @Override
  public void initialize() {
    subsystem.setArms(extend);
  }
}
