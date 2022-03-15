package frc.robot.commands.shooterTop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterTop;

public class WaitForShooterTopVelocity extends CommandBase {
  public final ShooterTop subsystem = RobotContainer.SHOOTER_TOP;
  
  @Override
  public boolean isFinished() {
    return subsystem.isStable(); 
  }
}
