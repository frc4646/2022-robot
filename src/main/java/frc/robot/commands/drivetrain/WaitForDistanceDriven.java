package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class WaitForDistanceDriven extends CommandBase {
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final double distance;

  public WaitForDistanceDriven(double distance) {
    this.distance = distance;
  }

  @Override
  public boolean isFinished() {
    return drive.getDistance() >= distance;
  }
}
