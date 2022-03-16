package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climber.ClimberOpenLoop;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;

public class HumanPlayerThenShoot extends SequentialCommandGroup {
  public HumanPlayerThenShoot(Trajectory pathHumanPlayer, Trajectory pathShoot3And4) {
    addCommands(
      deadline(
        new DrivePath(pathHumanPlayer),
        new WaitCommand(0.25).andThen(new DeployIntake())
      ),
      parallel(
        new DrivePath(pathShoot3And4),
        new IntakeExtend(false).andThen(new IntakeActivate(0.0))
      ),
      new DriveOpenLoop(0.0),
      new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM),
      parallel(
        new ShootVision(),
        new ClimberOpenLoop(1).beforeStarting(new WaitCommand(0.75))
      ),
      new DriveOpenLoop(0.1),
      new WaitCommand(0.5),
      new ClimberOpenLoop(-1),
      new WaitCommand(0.5)
    );
  }
}
