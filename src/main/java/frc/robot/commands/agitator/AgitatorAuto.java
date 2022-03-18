package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;

public class AgitatorAuto extends CommandBase {
  private final Agitator subsystem = RobotContainer.AGITATOR;

  public AgitatorAuto() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    subsystem.setOpenLoop(0.0, 0.0);
  }
}
