package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterWaitForVelocity extends CommandBase {
  public final Shooter subsystem = RobotContainer.SHOOTER;
  
  @Override
  public boolean isFinished() {
    return subsystem.isStable(); 
  }
}
