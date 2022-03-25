package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.shooter.ShooterRev;
import frc.robot.commands.turret.TurretAim;

public class GrabCargo2ThenShoot extends SequentialCommandGroup {
  public GrabCargo2ThenShoot(Trajectory pathCargo2, double distanceShoot) {
    addCommands(
      deadline(
        sequence(
          deadline(
            race(
              new DrivePath(pathCargo2).beforeStarting(new WaitCommand(ModeBase.TIME_INTAKE_DEPLOY)),
              new WaitUntilCommand(RobotContainer.FEEDER::isHopperFull)  // End path early: We have both cargo
            ),
            new DeployIntake()  // During path: Prepare to grab cargo
          ),
          deadline(
            new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM),
            new DriveOpenLoop(),
            new IntakeExtend(false).andThen(new IntakeOpenLoop())  // Workaround: StowIntake not finishing
            // new StowIntake()
          )
        ),
        new TurretAim(), new ShooterRev().beforeStarting(new WaitCommand(0.25))  // During path: Prepare to shoot
      ),
      new ShootVision(true),
      new FeederOpenLoop()  // After shooting: Stop feeder
    );
  }
}
