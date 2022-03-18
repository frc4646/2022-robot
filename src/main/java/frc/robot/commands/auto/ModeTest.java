package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DrivePath;

public class ModeTest extends ModeBase {
  private static final TrajectoryConfig
    // FORWARDS = Constants.DRIVETRAIN.TRAJECTORY_CONFIG;
    FORWARDS = Constants.DRIVETRAIN.PATH_CONFIG_F_SLOW;

  private static final Pose2d
    POSE_START = new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    POSE_END = new Pose2d(4.25, -1.0, new Rotation2d(0.0)),
    POSE_CARGO_3 = new Pose2d(0.5, 3.30, new Rotation2d(75.0)),
    POSE_HUMAN = new Pose2d(0.52, 6.98, new Rotation2d(43.5)),
    POSE_SHOOT_4_5 = new Pose2d(0.32, 2.97, new Rotation2d(63.2));

  private static final Translation2d
    WAYPOINT_CARGO_2_A = new Translation2d(0.9, -0.6),
    WAYPOINT_CARGO_2_B = new Translation2d(1.5, 0.9),
    WAYPOINT_CARGO_3 = new Translation2d(-0.15, 2.2);

  public static final Trajectory
    PATH = TrajectoryGenerator.generateTrajectory(POSE_START, List.of(), POSE_END, FORWARDS),
    PATH_CARGO_3 = TrajectoryGenerator.generateTrajectory(POSE_START, List.of(WAYPOINT_CARGO_2_A, WAYPOINT_CARGO_2_B, WAYPOINT_CARGO_3), POSE_CARGO_3, FORWARDS),
    PATH_HUMAN = TrajectoryGenerator.generateTrajectory(POSE_CARGO_3, List.of(), POSE_HUMAN, FORWARDS),
    PATH_SHOOT_4_5 = TrajectoryGenerator.generateTrajectory(POSE_HUMAN, List.of(), POSE_SHOOT_4_5, FORWARDS);

  public ModeTest(boolean useVelocityMode) {
    // addCommands(
    //   new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(PATH.getInitialPose()); }),
    //   new DrivePath(PATH, useVelocityMode)
    // );
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(PATH.getInitialPose()); }),
      new DrivePath(PATH_CARGO_3, useVelocityMode)
      // new DrivePath(PATH_HUMAN, useVelocityMode),
      // new DrivePath(PATH_SHOOT_4_5, useVelocityMode)
    );
  }
}
