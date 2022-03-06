package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class WaitForDistanceDriven extends CommandBase {
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final double distance, initial;

  public WaitForDistanceDriven(double distance) {
    this.distance = distance;
    this.initial = 0;
  }
  public WaitForDistanceDriven(double distance, double initialDistance) {
    this.distance = distance;
    this.initial = initialDistance;
  }

  @Override
  public boolean isFinished() {
    return Math.abs(drive.getDistance() - initial) >= distance;
  }
}
