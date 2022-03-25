package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Climber;

public class ClimberTeleop extends CommandBase{
  private final Climber subsystem = RobotContainer.CLIMBER;
  private final OperatorControls operator = RobotContainer.CONTROLS.getOperator();

  public ClimberTeleop() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double setpoint = 0.0;
    double stick = operator.getClimberStick();

    if (Math.abs(stick) > Constants.CLIMBER.DEADBAND) {
      setpoint = stick;
    }
    if (setpoint > Constants.CLIMBER.DEADBAND) {
      setpoint = 1.0;  // Extending always 100%, less likely to get stuck
    }
    subsystem.setOpenLoop(setpoint, 0.0);
  }
}
