package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.commands.shooter.ShooterRev;
import frc.robot.commands.turret.TurretPosition;

public class RightFiveCargo extends ModeBase {
  private static final TrajectoryConfig
    FORWARDS = Constants.DRIVETRAIN.TRAJECTORY_CONFIG,
    BACKWARDS = Constants.DRIVETRAIN.TRAJECTORY_CONFIG_REVERSED,
    FORWARDS_SLOW = Constants.DRIVETRAIN.TRAJECTORY_CONFIG_SLOW;
    
  private static final Pose2d
    // POSE_START = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
    // POSE_CARGO_3 = new Pose2d(.3, 3.0, Rotation2d.fromDegrees(95.0)),
    // POSE_HUMAN_PLAYER = new Pose2d(0.78, 6.9, Rotation2d.fromDegrees(40)),
    // POSE_SHOOT_4_5 = new Pose2d(0.1, 3.6, Rotation2d.fromDegrees(82.0));
    POSE_START = new Pose2d(7.8, 6.3, Rotation2d.fromDegrees(91.5)),
    POSE_CARGO_3 = new Pose2d(POSE_START.getX() + 3.2, POSE_START.getY() + 0.3, Rotation2d.fromDegrees(POSE_START.getRotation().getDegrees() - 85.0)),
    POSE_HUMAN_PLAYER = new Pose2d(POSE_START.getX() + 7.1, POSE_START.getY() + 0.78, Rotation2d.fromDegrees(POSE_START.getRotation().getDegrees() - 40)),
    POSE_SHOOT_4_5 = new Pose2d(POSE_START.getX() + 3.8, POSE_START.getY() + 0.1, Rotation2d.fromDegrees(POSE_START.getRotation().getDegrees() - 82.0));

  private static final Translation2d
  // TRANSLATION_CARGO_2 = new Translation2d(1.05, 0.6);
    TRANSLATION_CARGO_2 = new Translation2d(POSE_START.getX() + .9, POSE_START.getY() + 1.2);

  public static final Trajectory
    CARGO_2_3 = TrajectoryGenerator.generateTrajectory(POSE_START, List.of(TRANSLATION_CARGO_2), POSE_CARGO_3, FORWARDS_SLOW),
    HUMAN_PLAYER = TrajectoryGenerator.generateTrajectory(POSE_CARGO_3, List.of(), POSE_HUMAN_PLAYER, FORWARDS),
    SHOOT_4And5 = TrajectoryGenerator.generateTrajectory(POSE_HUMAN_PLAYER, List.of(), POSE_SHOOT_4_5, BACKWARDS);
    
  public RightFiveCargo() {
    
    RobotContainer.DRIVETRAIN.putTrajectory("CARGO_2_3", CARGO_2_3);
    RobotContainer.DRIVETRAIN.putTrajectory("HUMAN_PLAYER", HUMAN_PLAYER);
    RobotContainer.DRIVETRAIN.putTrajectory("SHOOT_4And5", SHOOT_4And5);
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(CARGO_2_3.getInitialPose()); }),

      // grab the 2nd cargo, continuing on to the 3rd
      // but be shooting when you get to the end
      // so you don't pull the penalty (momentary contact < 3 seconds)
      deadline(
        sequence(
          new WaitCommand(ModeBase.TIME_INTAKE_DEPLOY),
          new DrivePath(CARGO_2_3)
        ),
        new DeployIntake(),
        // sequence(
          new WaitCommand(1.5).andThen(new TurretPosition(215.0, 5.0)), // TODO refactor out const
          new WaitCommand(CARGO_2_3.getTotalTimeSeconds() - 3).andThen(new ShooterRev(150))//,
          // new WaitCommand(CARGO_2_3.getTotalTimeSeconds() - 1).andThen(new ShootVision()) 
        // )
      ),
      parallel(
        new DriveOpenLoop(0.0),
        new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM),
        new IntakeExtend(false).andThen(new IntakeOpenLoop(0.0))  // Workaround: StowIntake not finishing
      ),
      new ShootVision(), // todo how to shoot 3rd ball?
      parallel(
        new ShooterOpenLoop(0.0),
        new FeederOpenLoop(0.0)
      ),
      
      new HumanPlayerThenShoot(HUMAN_PLAYER, SHOOT_4And5)
    );
  }
}
