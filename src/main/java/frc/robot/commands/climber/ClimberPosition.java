package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberPosition extends CommandBase {
  private Climber subsystem = RobotContainer.CLIMBER;
  private final double position;

  public ClimberPosition(double position) {
    addRequirements(subsystem);
    this.position = position;
  }

  @Override
  public void initialize() {
    subsystem.setClosedLoopPosition(position);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isStable();
  }
}
