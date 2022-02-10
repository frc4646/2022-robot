package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretAutoZero extends CommandBase {
  private final Turret subsystem = RobotContainer.TURRET;

  public TurretAutoZero() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    double position = subsystem.getPositionTicks();
    // if (position < Constants.Turret.ENCODER_ZERO_MIN) {
    //   subsystem.setOpenLoop(0.05);
    // }
    // else if (position > Constants.Turret.ENCODER_ZERO_MAX) {
    //   subsystem.setOpenLoop(-0.05);
    // }
    // else {
    //   // Do nothing - Already at zero position
    // }
  }

  @Override
  public boolean isFinished() {
    double position = subsystem.getPositionTicks();
    // return position >= Constants.Turret.ENCODER_ZERO_MIN || position <= Constants.Turret.ENCODER_ZERO_MAX;
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setOpenLoop(0.0);
    subsystem.zeroSensors();
  }
}
