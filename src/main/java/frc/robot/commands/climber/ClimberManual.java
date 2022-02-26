package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberManual extends CommandBase {
  private final Climber subsystem = RobotContainer.CLIMBER;

  public ClimberManual() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double setpoint = RobotContainer.CONTROLS.operator.getClimberStick();
    subsystem.setOpenLoop(setpoint);
  }
}
