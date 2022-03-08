package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveTrajectory;

public class TestAuto extends SequentialCommandGroup {
  /**
   * Start: Origin, facing forwards (+X direction)
   * Waypoints: zig zag side to side (+/-Y direction) on path s curve
   * End: 3m straight ahead, facing forward
   */
  private final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    // List.of(),
    // new Pose2d(1.0, 0.0, new Rotation2d(0.0)),
    List.of(new Translation2d(1.0, 0.75), new Translation2d(2.0, -0.75)),
    new Pose2d(3.0, 0.0, new Rotation2d(0.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );
  // private final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
  //   new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
  //   List.of(),
  //   new Pose2d(1.0, 0.0, new Rotation2d(0.0)),
  //   Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  // );

  public TestAuto() {
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(trajectory.getInitialPose()); }),
      new DriveTrajectory(trajectory)
    );
  }
}
