package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterWaitForVelocity extends CommandBase {
  public final Shooter subsystem = RobotContainer.SHOOTER;

  public final double rpm;

  public ShooterWaitForVelocity(double wantedRPM) {
    addRequirements(subsystem);
    rpm = wantedRPM;
  }

  @Override
  public void initialize() {
    subsystem.setTargetRPM(rpm);
  }
  
  @Override
  public boolean isFinished() {
    return subsystem.isStable(); 
  }
}
