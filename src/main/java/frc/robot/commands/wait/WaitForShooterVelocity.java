package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;

public class WaitForShooterVelocity extends CommandBase {
  public final Shooter shooter = RobotContainer.SHOOTER;
  public final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  
  @Override
  public boolean isFinished() {
    return shooter.isStable() && shooterTop.isStable(); 
  }
}
