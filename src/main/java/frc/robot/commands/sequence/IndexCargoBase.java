package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Feeder;

public class IndexCargoBase extends CommandBase {
  private final Agitator agitator = RobotContainer.AGITATOR;
  private final Feeder feeder = RobotContainer.FEEDER;
  private final boolean on;

  public IndexCargoBase(boolean on) {
    addRequirements(agitator, feeder);
    this.on = on;
  }

  @Override
  public void initialize() {
    double setpointAgitator = on ? Constants.AGITATOR.OPEN_LOOP_LOAD : 0.0;
    double setpointFeeder = on ? Constants.FEEDER.OPEN_LOOP_LOAD : 0.0;
    agitator.setOpenLoop(setpointAgitator, setpointAgitator);
    feeder.setOpenLoop(setpointFeeder);
  }

  @Override
  public void end(boolean interrupted) {
    agitator.setOpenLoop(0.0, 0.0);
    feeder.setOpenLoop(0.0);
  }
}
