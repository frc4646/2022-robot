package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class WaitForDistanceDriven extends CommandBase {
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final double distanceRelativeWanted;
  private double distanceInitial = 0.0;

  public WaitForDistanceDriven(double distance) {
    this.distanceRelativeWanted = distance;
  }

  @Override
  public void initialize() {
    distanceInitial = drive.getDistance();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(drive.getDistance() - distanceInitial) >= distanceRelativeWanted;
  }
}
