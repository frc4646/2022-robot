package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Feeder;

public class AgitatorAuto extends CommandBase {
  private Agitator subsystem = RobotContainer.AGITATOR;
  private Feeder feeder = RobotContainer.FEEDER;

  private final double output;

  public AgitatorAuto(double percent) {
    addRequirements(subsystem);
    output = percent;
  }

  @Override
  public void execute() {
    double demand = (feeder.isBallPresent()) ? 0.0 : output;
    subsystem.setOpenLoop(demand);
  }
}
