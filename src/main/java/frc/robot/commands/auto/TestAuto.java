package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
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
  private static final double
    TIME_INTAKE_DEPLOY = 0.2,
    TIME_CANCEL_MOMENTUM = 0.2;

  private final TrajectoryConfig
    configForwards = Constants.DRIVETRAIN.TRAJECTORY_CONFIG,
    configBackwards = Constants.DRIVETRAIN.TRAJECTORY_CONFIG_REVERSED;

  private final Pose2d
    poseStart = new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    poseCargo2 = new Pose2d(1.5, 0.0, new Rotation2d(0.0)),
    poseHumanPlayer = new Pose2d(5.0, 0.8, Rotation2d.fromDegrees(-12.0));
    // poseAllianceWall = new Pose2d(5.15, 2.5, Rotation2d.fromDegrees(20.0));

  private final Trajectory
    pathCargo2 = TrajectoryGenerator.generateTrajectory(poseStart, List.of(), poseCargo2, configForwards),
    pathHumanPlayer = TrajectoryGenerator.generateTrajectory(poseCargo2, List.of(), poseHumanPlayer, configForwards),
    pathShoot3And4 = TrajectoryGenerator.generateTrajectory(poseHumanPlayer, List.of(), poseCargo2, configBackwards);
    // pathShoot1And2 = TrajectoryGenerator.generateTrajectory(poseCargo2, List.of(), new Pose2d(1.0, 0.0, new Rotation2d(0.0)), configBackwards),
    // pathAllianceWall = TrajectoryGenerator.generateTrajectory(poseCargo2, List.of(), poseAllianceWall, configForwards);

  public TestAuto() {
    addCommands(
      parallel(
        new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(pathCargo2.getInitialPose()); }), 
        new DeployIntake()
      ),
      new WaitCommand(TIME_INTAKE_DEPLOY),
      deadline(
        race(
          new DrivePath(pathCargo2),
          new WaitForColorState(STATE.CORRECT)
        ),
        new WaitCommand(0.25).andThen(new ShooterRev(115.0))
      ),
      parallel(
        new WaitCommand(TIME_CANCEL_MOMENTUM),
        new IntakeExtend(false).andThen(new IntakeActivate(0.0))  // Workaround: StowIntake not finishing
      ),
      new ShootVision(),
      parallel(
        new ShooterOpenLoop(0.0)
        // new FeederOpenLoop(0.0)
      ),
      deadline(
        new DrivePath(pathHumanPlayer),
        new WaitCommand(0.25).andThen(new DeployIntake())
      ),
      new DrivePath(pathShoot3And4),
      parallel(
        new WaitCommand(TIME_CANCEL_MOMENTUM),
        new IntakeExtend(false).andThen(new IntakeActivate(0.0))
      ),
      new ShootVision()
    );
  }

  @Override
  public void end(boolean isInterrupted) {
    RobotContainer.AGITATOR.setOpenLoop(0.0);
    RobotContainer.FEEDER.setOpenLoop(0.0);
    RobotContainer.INTAKE.setExtend(false);
    RobotContainer.INTAKE.setOpenLoop(0.0);
    RobotContainer.SHOOTER.setOpenLoop(0.0);
  }
}
