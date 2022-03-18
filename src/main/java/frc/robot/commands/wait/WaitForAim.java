package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Turret;

public class WaitForAim extends CommandBase {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final Turret turret = RobotContainer.TURRET;

  @Override
  public boolean isFinished() {
    return shooter.isStable() && shooterTop.isStable() && turret.isOnTarget();
  }
}
