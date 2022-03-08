package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DriveTrajectory;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.sequence.StowIntake;

public class TwoCargoAutoTrajectory extends SequentialCommandGroup {
  private final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    List.of(),
    new Pose2d(1.0, 0.0, new Rotation2d(0.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );

  public TwoCargoAutoTrajectory() {
  
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(trajectory.getInitialPose()); }),
      new DeployIntake(),

      new DriveTrajectory(trajectory),

      new StowIntake(),
      new ShootVision()
    );
  }
}
