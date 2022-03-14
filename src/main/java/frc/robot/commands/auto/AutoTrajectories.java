package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

/**
 * These are hand crafted, artisenal trajectories
 */
public class AutoTrajectories {
  
  public static final Trajectory midGrabCargo = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    List.of(
      // new Translation2d(1.0, 0.75),
    ),
    new Pose2d(1.8, 0.0, new Rotation2d(0.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );

  public static final Trajectory midShoot = TrajectoryGenerator.generateTrajectory(
    new Pose2d(1.8, 0.0, new Rotation2d(0.0)),
    List.of(
      // new Translation2d(1.0, 0.75),
    ),
    new Pose2d(1.0, 0.0, new Rotation2d(0.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG_REVERSED
  );

  public static final Trajectory midCargoToAllianceWall = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    List.of(
      // new Translation2d(1.0, 0.75),
    ),
    new Pose2d(5.15, 2.5, Rotation2d.fromDegrees(20.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );

  public static final Trajectory midCargoToHumanPlayer = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    List.of(
      // new Translation2d(1.0, 0.75),
    ),
    new Pose2d(5.0, .8, Rotation2d.fromDegrees(-12.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );

}
