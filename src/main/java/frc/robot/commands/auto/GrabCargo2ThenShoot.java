package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.feeder.WaitForColorState;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.commands.shooter.ShooterRev;
import frc.robot.subsystems.ColorSensor.STATE;

public class GrabCargo2ThenShoot extends SequentialCommandGroup {
  public GrabCargo2ThenShoot(Trajectory pathCargo2, double distanceShoot) {
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(pathCargo2.getInitialPose()); }),
      deadline(
        sequence(
          new WaitCommand(ModeBase.TIME_INTAKE_DEPLOY),
          race(
            new DrivePath(pathCargo2),
            new WaitForColorState(STATE.CORRECT)
          )
        ),
        new DeployIntake(),
        new WaitCommand(0.25).andThen(new ShooterRev(distanceShoot))  // TODO refactor out const
      ),
      parallel(
        new DriveOpenLoop(0.0),
        new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM),
        new IntakeExtend(false).andThen(new IntakeOpenLoop(0.0))  // Workaround: StowIntake not finishing
      ),
      new ShootVision(),
      parallel(
        new ShooterOpenLoop(0.0),
        new FeederOpenLoop(0.0)
      )
    );
  }
}
