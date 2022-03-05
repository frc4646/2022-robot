package frc.robot.commands.agitator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AgitatorAuto extends CommandBase {
  private Agitator subsystem = RobotContainer.AGITATOR;
  private Climber climber = RobotContainer.CLIMBER;
  private Intake intake = RobotContainer.INTAKE;
  private Shooter shooter = RobotContainer.SHOOTER;

  public AgitatorAuto() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double demand = 0.0;

    if (climber.isInClimbMode()) {
      demand = 0.0;
    } else if (shooter.isShooting()) {
      demand = Constants.AGITATOR.OPEN_LOOP_SHOOT;
    } else if (intake.isExtended()) {
      demand = Constants.AGITATOR.OPEN_LOOP_LOAD;
    }
    subsystem.setOpenLoop(demand);
  }
}
