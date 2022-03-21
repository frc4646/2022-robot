package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotState;

public class WaitForShooterVelocity extends CommandBase {
  public final RobotState state = RobotContainer.ROBOT_STATE;
  
  @Override
  public boolean isFinished() {
    return state.isShootingStable();
  }
}
