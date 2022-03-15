package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Vision;

public class ShooterAuto extends CommandBase {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  private final Feeder feeder = RobotContainer.FEEDER;
  private final Vision vision = RobotContainer.VISION;
  private SlewRateLimiter limiter = new SlewRateLimiter(1.0);

  public ShooterAuto() {
    addRequirements(shooter, shooterTop);
  }

  @Override
  public void initialize() {
    limiter = new SlewRateLimiter(1.0);
  }

  @Override
  public void execute() {
    double setpoint = 0.0;

    // if (feeder.isShooterLoaded() && vision.isInShootRange()) {
    //   setpoint = vision.getShooterRPM() * 0.2;
    // }
    // TODO if loaded cargo wrong, use exhaust rpm?
    // TODO if both cargo correct, prepare using minimum viable rpm?

    // setpoint = limiter.calculate(setpoint);
    shooter.setClosedLoop(setpoint);
    shooterTop.setClosedLoop(setpoint);
  }
}
