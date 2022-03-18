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

public class HumanPlayerThenShoot extends SequentialCommandGroup {
  public HumanPlayerThenShoot(Trajectory pathHumanPlayer, Trajectory pathShoot3And4) {
    addCommands(
      deadline(
        new DrivePath(pathHumanPlayer),
        new DeployIntake().beforeStarting(new WaitCommand(0.25))  // TODO wait for distance driven
      ),
      parallel(
        new DrivePath(pathShoot3And4),
        // TODO stop early if has two cargo
        new IntakeExtend(false).andThen(new IntakeOpenLoop())  // TODO stow
      ),
      new DriveOpenLoop(),
      new ShootVision().beforeStarting(new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM))  // TODO wait for drive speed
    );
  }
}
