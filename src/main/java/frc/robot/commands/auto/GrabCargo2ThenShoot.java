package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeExtend;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.commands.shooter.ShooterRev;
import frc.robot.commands.wait.WaitForColorState;
import frc.robot.subsystems.ColorSensor.STATE;

public class GrabCargo2ThenShoot extends SequentialCommandGroup {
  public GrabCargo2ThenShoot(Trajectory pathCargo2, double distanceShoot) {
    addCommands(
      deadline(
        race(
          new DrivePath(pathCargo2).beforeStarting(new WaitCommand(ModeBase.TIME_INTAKE_DEPLOY)),
          new WaitForColorState(STATE.CORRECT)
        ),
        new DeployIntake(),
        new ShooterRev().beforeStarting(new WaitCommand(0.25))  // TODO refactor out const
        // TODO hint turret to shoot angle
        // TODO ScheduleCommand the turret hint so default command happens afterwards?
      ),
      deadline(
        new WaitCommand(ModeBase.TIME_CANCEL_MOMENTUM),  // TODO wait for drive speed
        new DriveOpenLoop(),
        new IntakeExtend(false).andThen(new IntakeOpenLoop())  // Workaround: StowIntake not finishing
        // TODO update rev rpm further while slowing down new ShooterRev()
      ),
      new ShootVision(),
      parallel(new ShooterOpenLoop(), new FeederOpenLoop())
    );
  }
}
