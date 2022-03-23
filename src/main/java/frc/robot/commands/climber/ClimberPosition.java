package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimberPosition extends CommandBase {
  private Climber subsystem = RobotContainer.CLIMBER;
  private final double percentUp;

  public ClimberPosition(double percentUp) {
    addRequirements(subsystem);
    this.percentUp = percentUp;
  }

  @Override
  public void initialize() {
    subsystem.setClosedLoopPosition(percentUp, 0.0);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isStable();
  }
}
