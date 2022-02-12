package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.team254.util.InterpolatingDouble;

public class ShooterRevDistance extends CommandBase {
  public final Shooter subsystem = RobotContainer.SHOOTER;
  public final double distance;

  public ShooterRevDistance(double distance) {
    addRequirements(subsystem);
    this.distance = distance;
  }

  @Override
  public void initialize() {
    double rpm = Constants.Shooter.RPM_MAP.getInterpolated(new InterpolatingDouble(distance)).value;
    subsystem.setClosedLoop(rpm);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isStable();
  }
}
