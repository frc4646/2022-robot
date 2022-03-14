package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.drivetrain.DriveTrajectory;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.feeder.WaitForColorState;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.commands.shooter.ShooterRev;
import frc.robot.subsystems.ColorSensor.STATE;

public class TestAuto extends SequentialCommandGroup {
  private static final double X_SECOND_CARGO = 1.5;
  private static final double X_HUMAN_PLAYER = 5.0;
  private static final double TIME_INTAKE_DEPLOY = 0.2;
  private static final double TIME_DRIVE_CANCEL_MOMENTUM = 0.2;

  private final Trajectory trajectoryGrabCargo = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    List.of(),
    new Pose2d(X_SECOND_CARGO, 0.0, new Rotation2d(0.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );
  private final Trajectory trajectoryShoot = TrajectoryGenerator.generateTrajectory(
    new Pose2d(X_SECOND_CARGO, 0.0, new Rotation2d(0.0)),
    List.of(),
    new Pose2d(1.0, 0.0, new Rotation2d(0.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG_REVERSED
  );
  private final Trajectory trajectoryToAllianceWall = TrajectoryGenerator.generateTrajectory(
    new Pose2d(X_SECOND_CARGO, 0.0, new Rotation2d(0.0)),
    List.of(),
    new Pose2d(5.15, 2.5, Rotation2d.fromDegrees(20.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );
  private final Trajectory trajectoryToHumanPlayer = TrajectoryGenerator.generateTrajectory(
    new Pose2d(X_SECOND_CARGO, 0.0, new Rotation2d(0.0)),
    List.of(),
    new Pose2d(X_HUMAN_PLAYER, .8, Rotation2d.fromDegrees(-12.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );

  public TestAuto() {
    addCommands(
      parallel(
        new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(trajectoryGrabCargo.getInitialPose()); }),   
        new DeployIntake()
      ),
      new WaitCommand(TIME_INTAKE_DEPLOY),
      deadline(
        race(
          new DriveTrajectory(trajectoryGrabCargo),
          new WaitForColorState(STATE.CORRECT)  // Possibly stop early
        ),
        new WaitCommand(0.25).andThen(new ShooterRev(115.0))
      ),
      // new DriveTrajectory(trajectoryShoot),
      parallel(
        new WaitCommand(TIME_DRIVE_CANCEL_MOMENTUM),
        sequence(
          new IntakeExtend(false),
          new IntakeActivate(0.0)
        )
      ),
      new ShootVision(),
      parallel(
        new ShooterOpenLoop(0.0)
        // new FeederOpenLoop(0.0)
      ),
      // new DriveTrajectory(trajectoryToAllianceWall)
      deadline(
        new DriveTrajectory(trajectoryToHumanPlayer),
        new WaitCommand(0.25).andThen(new DeployIntake())
      )
    );
  }  

  @Override
  public void end(boolean isInterrupted) {
    RobotContainer.FEEDER.setOpenLoop(0.0);
    RobotContainer.AGITATOR.setOpenLoop(0.0);
    RobotContainer.INTAKE.setExtend(false);
    RobotContainer.INTAKE.setOpenLoop(0.0);
  }
}
