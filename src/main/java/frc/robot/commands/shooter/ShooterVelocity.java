package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;

public class ShooterVelocity extends InstantCommand {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final double rpm, rpmTop;

  public ShooterVelocity(double rpm, double rpmTop) {
    addRequirements(shooter, shooterTop);
    this.rpm = rpm;
    this.rpmTop = rpmTop;
  }

  @Override
  public void initialize() {
    shooter.setClosedLoop(rpm);
    shooterTop.setClosedLoop(rpmTop);
  }
}
