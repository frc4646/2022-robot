package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class WaitForAim extends CommandBase {
  // private final Hood hood = RobotContainer.HOOD;
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final Turret turret = RobotContainer.TURRET;
  private double timeStart = Double.MAX_VALUE;

  @Override
  public void initialize() {
    timeStart = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    if (RobotContainer.CONTROLS.operator.getFn()) {  // Manual override
      return Timer.getFPGATimestamp() - timeStart > Constants.Shooter.OPEN_LOOP_REV_SECONDS;
    }
    // return shooter.isStable() && hood.isOnTarget() && turret.isOnTarget();
    return shooter.isStable() && turret.isOnTarget();
  }
}
