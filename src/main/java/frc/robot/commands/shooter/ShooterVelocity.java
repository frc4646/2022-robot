package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.util.ShootSetpoint;

public class ShooterVelocity extends InstantCommand {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final ShootSetpoint setpoint;

  public ShooterVelocity(ShootSetpoint setpoint) {
    addRequirements(shooter, shooterTop);
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {
    shooter.setClosedLoop(setpoint, true);
    shooterTop.setClosedLoop(setpoint);
  }
}
