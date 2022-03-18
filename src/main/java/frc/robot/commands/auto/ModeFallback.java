package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.drivetrain.DriveOpenLoop;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.sequence.StowIntake;
import frc.robot.commands.wait.WaitForColorState;
import frc.robot.commands.wait.WaitForDistanceDriven;
import frc.robot.subsystems.ColorSensor.STATE;

/** Time based grab a cargo and shoot it */
public class ModeFallback extends ModeBase {
  public ModeFallback() {
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0))); }),
   
      new DeployIntake(),
      new DriveOpenLoop(.15).beforeStarting(new WaitCommand(1.0)),
      new WaitForDistanceDriven(1.8).withTimeout(1.5),
      new WaitCommand(.2),
      parallel(
        sequence(
          new DriveOpenLoop(-.15),
          new DriveOpenLoop().beforeStarting(new WaitCommand(.65))
        ),
        sequence(
          new WaitForColorState(STATE.CORRECT).withTimeout(2.0),
          new AgitateOpenLoop(),
          new StowIntake()
        )
      ),
      new ShootVision()
    );
  }
}
