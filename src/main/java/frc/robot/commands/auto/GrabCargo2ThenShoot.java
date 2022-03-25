package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.agitator.AgitatorAuto;
import frc.robot.commands.agitator.AgitatorOpenLoop;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
import frc.robot.commands.feeder.FeederAutoIndex;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.sequence.StowIntake;
import frc.robot.commands.shooter.ShooterAutoRev;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.commands.shooter.ShooterRev;
import frc.robot.commands.turret.TurretAim;
import frc.robot.commands.turret.TurretAutoAim;

public class GrabCargo2ThenShoot extends SequentialCommandGroup {
  public GrabCargo2ThenShoot(Trajectory pathCargo2, double distanceShoot) {
    addCommands(
      deadline(
        sequence(
          deadline(
            race(
              new DrivePath(pathCargo2).beforeStarting(new WaitCommand(ModeBase.TIME_INTAKE_DEPLOY)),
              new WaitUntilCommand(RobotContainer.FEEDER::isHopperFull)
            ),
            new DeployIntake()
          ),
          deadline(
            new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM),  // TODO wait for drive speed
            new DriveOpenLoop(),
            new IntakeExtend(false).andThen(new IntakeOpenLoop())  // Workaround: StowIntake not finishing
            // new StowIntake()
          )
        ),
        new TurretAim(),
        new ShooterRev().beforeStarting(new WaitCommand(0.25))
      ),
      new ShootVision(true),
      new FeederOpenLoop()
    );
  }
}
