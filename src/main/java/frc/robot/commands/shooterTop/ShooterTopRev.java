package frc.robot.commands.shooterTop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterTop;
import frc.team254.util.InterpolatingDouble;

public class ShooterTopRev extends InstantCommand{
  private ShooterTop subsystem = RobotContainer.SHOOTER_TOP;
  private final double distance;

  public ShooterTopRev(double distance) {
    addRequirements(subsystem);
    this.distance = distance;
  }

  @Override
  public void initialize() {
    double rpm = Constants.VISION.RPM_TOP.getInterpolated(new InterpolatingDouble(distance)).value;
    subsystem.setClosedLoop(rpm);
  }
}
