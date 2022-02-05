package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class AgitatorAuto extends CommandBase {
  private Agitator subsystem = RobotContainer.AGITATOR;
  private Feeder feeder = RobotContainer.FEEDER;
  private Intake intake = RobotContainer.INTAKE;

  private final double output;

  public AgitatorAuto(double percent) {
    addRequirements(subsystem);
    output = percent;
  }

  @Override
  public void execute() {
    //double demand = (feeder.isBallPresent()) ? 0.0 : output;
    double demand = (intake.isExtended()) ? output : 0.25;
    subsystem.setOpenLoop(demand);
  }
}
