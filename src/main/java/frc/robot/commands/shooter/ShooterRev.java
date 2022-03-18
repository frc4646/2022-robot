package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;

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
    shooter.setClosedLoop(Constants.VISION.shootTree.getRPMBottom(distance));
    shooterTop.setClosedLoop(Constants.VISION.shootTree.getRPMTop(distance));
  }
}
