package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTop;

public class ShooterOpenLoop extends InstantCommand {
  private final Shooter shooter = RobotContainer.SHOOTER;
  private final ShooterTop shooterTop = RobotContainer.SHOOTER_TOP;
  public final double output;

  public ShooterOpenLoop(double percent) {
    addRequirements(shooter, shooterTop);
    this.output = percent;
  }

  @Override
  public void initialize() {
    shooter.setOpenLoop(output);
    shooterTop.setOpenLoop(output*2.0); //TODO constant?
  }
}
