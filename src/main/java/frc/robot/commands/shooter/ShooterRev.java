package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.team254.util.InterpolatingDouble;

public class ShooterRev extends InstantCommand {
  private Shooter subsystem = RobotContainer.SHOOTER;
  private final double distance;

  public ShooterRev(double distance) {
    addRequirements(subsystem);
    this.distance = distance;
  }

  @Override
  public void initialize() {
        double rpm = Constants.VISION.RPM_BOTTOM.getInterpolated(new InterpolatingDouble(distance)).value;
    subsystem.setClosedLoop(rpm);
  }
}
