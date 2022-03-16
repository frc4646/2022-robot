package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.drivetrain.DriveOpenLoop;
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

public class TestAutoPathweaver extends ModeBase {
  private final Trajectory pathCargo = GeneratedPaths.Right_RightCargo_MidCargo;
  private final Trajectory pathHumanPlayer = GeneratedPaths.MidCargoFromRight_Terminal;
  private final Trajectory pathShoot = GeneratedPaths.Terminal_Shoot;

  public TestAutoPathweaver() {
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(pathCargo.getInitialPose()); }),

      // grab the 2nd cargo, continuing on to the 3rd
      // but be shooting when you get to the end
      // so you don't pull the penalty (momentary contact < 3 seconds)
      deadline(
        sequence(
          new WaitCommand(ModeBase.TIME_INTAKE_DEPLOY),
          new DrivePath(pathCargo)
        ),
        new DeployIntake(),
        new WaitCommand(1.5).andThen(new ShooterRev(150))  // TODO refactor out const
      ),
      parallel(
        new DriveOpenLoop(0.0),
        new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM),
        new IntakeExtend(false).andThen(new IntakeActivate(0.0))  // Workaround: StowIntake not finishing
      ),
      new ShootVision(),
      parallel(
        new ShooterOpenLoop(0.0),
        new FeederOpenLoop(0.0)
      ),
      
      new HumanPlayerThenShoot(pathHumanPlayer, pathShoot)
    );
  }
}
