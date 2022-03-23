package frc.robot.commands.wait;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class WaitForPose extends CommandBase {
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private final Translation2d wanted;
  private final double toleranceMeters;

  public WaitForPose(Pose2d wanted, double toleranceMeters) {
    this(wanted.getTranslation(), toleranceMeters);
  }

  public WaitForPose(Translation2d wanted, double toleranceMeters) {
    this.wanted = wanted;
    this.toleranceMeters = toleranceMeters;
  }

  @Override
  public boolean isFinished() {
    return drive.getPose().getTranslation().getDistance(wanted) <= toleranceMeters;
  }
}
