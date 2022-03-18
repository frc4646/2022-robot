package frc.robot.commands.wait;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class WaitForDriveVelocity extends CommandBase {
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final double speedWanted;

  public WaitForDriveVelocity(double metersPerSecond) {
    this.speedWanted = metersPerSecond;
  }

  @Override
  public boolean isFinished() {
    DifferentialDriveWheelSpeeds speed = drive.getWheelSpeeds();
    return Math.abs(speed.leftMetersPerSecond + speed.rightMetersPerSecond) / 2.0 <= speedWanted;
  }
}
