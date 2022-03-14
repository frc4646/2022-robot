package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.drivetrain.WaitForDistanceDriven;
import frc.robot.commands.feeder.WaitForColorState;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.sequence.StowIntake;
import frc.robot.subsystems.ColorSensor.STATE;

/**
 * Time based grab a cargo and shoot it.
 * Not flexible enough for all 3 positions
 */
public class TwoCargoAuto extends SequentialCommandGroup {
  public TwoCargoAuto() {
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0))); }),
   
      new DeployIntake(),
      new WaitCommand(1.0),
      new DriveOpenLoop(.15),
      new WaitForDistanceDriven(1.8).withTimeout(1.5),
      new WaitCommand(.2),
      parallel(
        sequence(
          new DriveOpenLoop(-.15),
          new WaitCommand(.65),
          new DriveOpenLoop(0.0)
        ),
        sequence(
          new WaitForColorState(STATE.CORRECT).withTimeout(2.0),
          new AgitateOpenLoop(0.0),
          new StowIntake()
        )
      ),
      new ShootVision()
    );
  }

  @Override
  public void end(boolean isInterrupted) {
    RobotContainer.FEEDER.setOpenLoop(0.0);
    RobotContainer.AGITATOR.setOpenLoop(0.0);
    RobotContainer.INTAKE.setExtend(false);
    RobotContainer.INTAKE.setOpenLoop(0.0);
  }
}
