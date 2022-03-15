package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;

public class ShooterPairTune extends CommandBase {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;

  public ShooterPairTune() {
    addRequirements(shooter, shooterTop);
    SmartDashboard.putNumber("ShooterPairTune Bottom", 2400.0);
  }

  @Override
  public void execute() {
    double rpm = SmartDashboard.getNumber("ShooterPairTune Bottom", 0.0);
    shooter.setClosedLoop(rpm);
    shooterTop.setClosedLoop(rpm * 2.0); //TODO constant?
  }
}
