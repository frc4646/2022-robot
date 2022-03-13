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
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DriveTrajectory;
import frc.robot.commands.drivetrain.WaitForDistanceDriven;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.feeder.WaitForColorState;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.sequence.StowIntake;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.subsystems.ColorSensor.STATE;

public class TestAuto extends SequentialCommandGroup {
  private final Trajectory trajectoryGrabCargo = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    List.of(
      // new Translation2d(1.0, 0.75),
    ),
    new Pose2d(1.8, 0.0, new Rotation2d(0.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );

  private final Trajectory trajectoryShoot = TrajectoryGenerator.generateTrajectory(
    new Pose2d(1.8, 0.0, new Rotation2d(0.0)),
    List.of(
      // new Translation2d(1.0, 0.75),
    ),
    new Pose2d(1.0, 0.0, new Rotation2d(0.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG_REVERSED
  );

  private final Trajectory trajectoryToAllianceWall = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    List.of(
      // new Translation2d(1.0, 0.75),
    ),
    new Pose2d(5.15, 2.5, Rotation2d.fromDegrees(20.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );

  private final Trajectory trajectoryToHumanPlayer = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
    List.of(
      // new Translation2d(1.0, 0.75),
    ),
    new Pose2d(5.0, .8, Rotation2d.fromDegrees(-12.0)),
    Constants.DRIVETRAIN.TRAJECTORY_CONFIG
  );

  public TestAuto() {
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(trajectoryGrabCargo.getInitialPose()); }),   
      new DeployIntake(),
      new WaitCommand(1.0),
        sequence(
          new DriveTrajectory(trajectoryGrabCargo)
      ),
      new WaitCommand(.2),
      parallel(        
        new DriveTrajectory(trajectoryShoot),
        // sequence(          
        //   new DriveOpenLoop(-.15),
        //   new WaitCommand(.65),
        //   new DriveOpenLoop(0.0)
        // ),
        sequence(
          new WaitForColorState(STATE.CORRECT).withTimeout(2.0),
          new AgitateOpenLoop(0.0)//,
          // new StowIntake()
        )
      ),      
      new ShootVision(),
      parallel(
        new FeederOpenLoop(0.0),
        // new DeployIntake(),
        new ShooterOpenLoop(0.0)
      ),
      // new DriveTrajectory(trajectoryToAllianceWall)
      new DriveTrajectory(trajectoryToHumanPlayer)
    );
  }  

  @Override
  public void end(boolean isInterrupted) {
    RobotContainer.FEEDER.setOpenLoop(0);
    RobotContainer.AGITATOR.setOpenLoop(0);
    RobotContainer.INTAKE.setExtend(false);
    RobotContainer.INTAKE.setIntakeSpeed(0);
  }
}
