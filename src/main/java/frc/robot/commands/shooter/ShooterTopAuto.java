package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;
import frc.robot.subsystems.Vision;

public class ShooterTopAuto extends CommandBase {
  private final ShooterTop subsystem = RobotContainer.SHOOTER_TOP;

  public ShooterTopAuto() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double setpoint = 0.0;
    subsystem.setClosedLoop(setpoint);
  }
}
