package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public abstract class ModeBase extends SequentialCommandGroup {
  public static enum STRATEGY_PHASE_2 {
    HUMAN_PLAYER, NONE
  }
  
  public static final double TIME_CANCEL_MOMENTUM = 0.2;
  public static final double TIME_INTAKE_DEPLOY = 0.2;

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    RobotContainer.AGITATOR.setOpenLoop(0.0, 0.0);
    RobotContainer.FEEDER.setOpenLoop(0.0);
    RobotContainer.INTAKE.setExtend(false);
    RobotContainer.INTAKE.setOpenLoop(0.0);
    RobotContainer.SHOOTER.setOpenLoop(0.0);
  }
}
