package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.team254.util.InterpolatingDouble;

public class ShooterRev extends InstantCommand {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final double distance;

  public ShooterRev(double distance) {
    addRequirements(shooter, shooterTop);
    this.distance = distance;
  }

  @Override
  public void initialize() {
    shooter.setClosedLoop(Constants.VISION.RPM_BOTTOM.getInterpolated(new InterpolatingDouble(distance)).value);
    shooterTop.setClosedLoop(Constants.VISION.RPM_TOP.getInterpolated(new InterpolatingDouble(distance)).value);
  }
}
