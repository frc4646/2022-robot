package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.agitator.AgitatorAuto;
import frc.robot.commands.agitator.AgitatorPulse;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
import frc.robot.commands.feeder.FeederAutoIndex;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.sequence.StowIntake;
import frc.robot.commands.shooter.ShooterRev;
import frc.robot.commands.turret.TurretAim;
import frc.robot.commands.wait.WaitForDistanceDriven;

public class HumanPlayerThenShoot extends SequentialCommandGroup {
  public HumanPlayerThenShoot(Trajectory pathHumanPlayer, Trajectory pathShoot3And4) {
    addCommands(
      // parallel(new AgitatorAuto(), new FeederAutoIndex(), new ShooterAutoRev())  // Reset default commands after shoot in autonomous mode
      deadline(
        sequence(
          deadline(
            new DrivePath(pathHumanPlayer),  // TODO stop early if has two cargo
            new DeployIntake().beforeStarting(new WaitForDistanceDriven(1.5)),
            new AgitatorPulse().beforeStarting(new WaitForDistanceDriven(1.5))
          ),
          deadline(
            sequence(
              deadline(new DrivePath(pathShoot3And4), new StowIntake(0.2)),
              deadline(new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM), new DriveOpenLoop())
            ),
            new AgitatorAuto(),
            new TurretAim(),
            new ShooterRev()
          )
        ),
        new FeederAutoIndex()
      ),
      new ShootVision(true)  // TODO wait for drive speed
    );
  }
}
