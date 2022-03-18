package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class FeederPosition extends CommandBase {
  private final Feeder subsystem = RobotContainer.FEEDER;
  private final double turnsRelative;

  public FeederPosition(double turnsRelative) {
    addRequirements(subsystem);
    this.turnsRelative = turnsRelative;
  }

  @Override
  public void initialize() {
    subsystem.setClosedLoopPosition(subsystem.getPosition() + turnsRelative);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isOnTarget();
  }
}
