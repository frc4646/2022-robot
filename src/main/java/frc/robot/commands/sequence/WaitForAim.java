package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class WaitForAim extends CommandBase {
  // private final Hood hood = RobotContainer.HOOD;
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final Turret turret = RobotContainer.TURRET;

  @Override
  public boolean isFinished() {
    // return shooter.isStable() && hood.isOnTarget() && turret.isOnTarget();
    return shooter.isStable() && turret.isOnTarget();
  }
}
