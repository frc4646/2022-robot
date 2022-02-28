package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberAuto extends CommandBase {
  private final Climber subsystem = RobotContainer.CLIMBER;

  public ClimberAuto() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double setpoint = 0.0;
    double stick = RobotContainer.CONTROLS.operator.getClimberStick();

    if (Math.abs(stick) > Constants.Climber.DEADBAND) {
      setpoint = stick;
    }
    subsystem.setOpenLoop(setpoint);
  }
}