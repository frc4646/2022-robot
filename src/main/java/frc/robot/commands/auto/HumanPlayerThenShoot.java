package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.wait.WaitForDistanceDriven;

public class HumanPlayerThenShoot extends SequentialCommandGroup {
  public HumanPlayerThenShoot(Trajectory pathHumanPlayer, Trajectory pathShoot3And4) {
    addCommands(
      deadline(
        new DrivePath(pathHumanPlayer),  // TODO stop early if has two cargo
        new DeployIntake().beforeStarting(new WaitForDistanceDriven(1.5))  // TODO tune
      ),
      deadline(
        new DrivePath(pathShoot3And4),
        new IntakeExtend(false).andThen(new IntakeOpenLoop())  // TODO stow after pose or distance driven instead
        // TODO new StowIntake().beforeStarting(new WaitForDistanceDriven(0.5))
        // TODO new ShooterRev().beforeStarting(new WaitForDistanceDriven(2.0))
      ),
      new DriveOpenLoop(),
      // TODO keep reving to refine correct RPM
      new ShootVision().beforeStarting(new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM))  // TODO wait for drive speed
    );
  }
}
