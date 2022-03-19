package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Vision;

public class ShooterRev extends CommandBase {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final Vision vision = RobotContainer.VISION;
  private final double distanceDefault;

  public ShooterRev() {
    this((Constants.VISION.MAP.getDistanceMax() + Constants.VISION.MAP.getDistanceMin()) / 2.0);
  }

  public ShooterRev(double distance) {
    addRequirements(shooter, shooterTop);
    this.distanceDefault = distance;
  }

  @Override
  public void execute() {
    double setpointB, setpointT;
    if (vision.isTargetPresent()) {
      setpointB = vision.getShooterRPMBottom();
      setpointT = vision.getShooterRPMTop();
    } else {
      setpointB = Constants.VISION.MAP.getRPMBottom(distanceDefault);
      setpointT = Constants.VISION.MAP.getRPMTop(distanceDefault);
    }
    shooter.setClosedLoop(setpointB, false);
    shooterTop.setClosedLoop(setpointT);
  }
}
