package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AgitatorAuto extends CommandBase {
  private Agitator subsystem = RobotContainer.AGITATOR;
  private Intake intake = RobotContainer.INTAKE;
  private Shooter shooter = RobotContainer.SHOOTER;

  public AgitatorAuto() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double demand = 0.0;

    if (shooter.isShooting()) {
      demand = Constants.Agitator.OPEN_LOOP_SHOOTING;
    } else if (intake.isExtended()) {
      demand = Constants.Agitator.OPEN_LOOP_LOADING;
    }
    subsystem.setOpenLoop(demand);
  }
}
