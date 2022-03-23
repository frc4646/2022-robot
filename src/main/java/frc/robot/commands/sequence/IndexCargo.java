package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Feeder;

public class IndexCargo extends CommandBase {
  private final Agitator agitator = RobotContainer.AGITATOR;
  private final Feeder feeder = RobotContainer.FEEDER;
  private final boolean feed, agitate;

  public IndexCargo() {
    this(true, true);
  }

  public IndexCargo(boolean feed, boolean agitate) {
    addRequirements(agitator, feeder);
    this.feed = feed;
    this.agitate = agitate;
  }

  @Override
  public void initialize() {
    double setpointAgitator = agitate ? Constants.AGITATOR.OPEN_LOOP_LOAD : 0.0;
    double setpointFeeder = feed ? Constants.FEEDER.OPEN_LOOP_LOAD : 0.0;
    agitator.setOpenLoop(setpointAgitator, setpointAgitator);
    feeder.setOpenLoop(setpointFeeder);
  }

  @Override
  public void end(boolean interrupted) {
    agitator.setOpenLoop(0.0, 0.0);
    feeder.setOpenLoop(0.0);
  }
}
