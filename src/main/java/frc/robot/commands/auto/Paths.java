package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;

/** These are hand crafted, artisenal trajectories */
public class Paths {
  private static final TrajectoryConfig
    FORWARDS = Constants.DRIVETRAIN.TRAJECTORY_CONFIG,
    BACKWARDS = Constants.DRIVETRAIN.TRAJECTORY_CONFIG_REVERSED;

  public static class LEFT {
  
  }

  public static class MIDDLE {
    private static final Pose2d
      POSE_START = new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
      POSE_CARGO_2 = new Pose2d(1.5, 0.0, new Rotation2d(0.0)),
      POSE_HUMAN_PLAYER = new Pose2d(5.0, 0.8, Rotation2d.fromDegrees(-12.0)),
      POSE_ALLIANCE_WALL = new Pose2d(5.15, 2.5, Rotation2d.fromDegrees(20.0));

    public static final Trajectory
      CARGO_2 = TrajectoryGenerator.generateTrajectory(POSE_START, List.of(), POSE_CARGO_2, FORWARDS),
      pathHumanPlayer = TrajectoryGenerator.generateTrajectory(POSE_CARGO_2, List.of(), POSE_HUMAN_PLAYER, FORWARDS),
      pathShoot3And4 = TrajectoryGenerator.generateTrajectory(POSE_HUMAN_PLAYER, List.of(), POSE_CARGO_2, BACKWARDS),
      pathAllianceWall = TrajectoryGenerator.generateTrajectory(POSE_CARGO_2, List.of(), POSE_ALLIANCE_WALL, FORWARDS);
  }

  public static class RIGHT {
    
  }
}
