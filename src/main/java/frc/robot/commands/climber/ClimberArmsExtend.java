package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberArms;

public class ClimberArmsExtend extends CommandBase {
  final ClimberArms subsystem = RobotContainer.CLIMBER_ARMS;
  final boolean extend;

  public ClimberArmsExtend(boolean extend) {
    addRequirements(subsystem);
    this.extend = extend;
  }

  @Override
  public void initialize() {
    subsystem.setArms(extend);
  }
}
