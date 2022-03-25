package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
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
  private static final double DISTANCE_DEPLOY_INTAKE = 1.5;
  private static final double STOW_INTAKE_LATER_THAN_NORMAL = 0.5;

  public HumanPlayerThenShoot(Trajectory pathHumanPlayer, Trajectory pathShoot3And4) {
    addCommands(
      deadline(
        sequence(
          deadline(
            new DrivePath(pathHumanPlayer),
            new DeployIntake().beforeStarting(new WaitForDistanceDriven(DISTANCE_DEPLOY_INTAKE)),
            new AgitatorPulse().beforeStarting(new WaitForDistanceDriven(DISTANCE_DEPLOY_INTAKE))
          ),
          deadline(
            sequence(
              deadline(new DrivePath(pathShoot3And4), new StowIntake(STOW_INTAKE_LATER_THAN_NORMAL)),
              deadline(new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM), new DriveOpenLoop())
            ),
            new AgitatorAuto(), new TurretAim(), new ShooterRev()  // During second path, prepare to shoot
          )
        ),
        new FeederAutoIndex()  // During both paths: Index cargos that enter the robot
      ),
      new ShootVision(true)  // After both paths, shoot
    );
  }
}
