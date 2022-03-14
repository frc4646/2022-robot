package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.drivetrain.DriveTrajectory;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.feeder.WaitForColorState;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.subsystems.ColorSensor.STATE;

/**
 * Saved copy of path testing from the scrimmage
 */
public class TwoCargoMidAndTerminal extends SequentialCommandGroup {
  public TwoCargoMidAndTerminal() {
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(AutoTrajectories.midGrabCargo.getInitialPose()); }),   
      new DeployIntake(),
      new WaitCommand(1.0),
        sequence(
          new DriveTrajectory(AutoTrajectories.midGrabCargo)
      ),
      new WaitCommand(.2),
      parallel(        
        new DriveTrajectory(AutoTrajectories.midShoot),
        sequence(
          new WaitForColorState(STATE.CORRECT).withTimeout(2.0),
          new AgitateOpenLoop(0.0)
        )
      ),      
      new ShootVision(),
      parallel(
        new FeederOpenLoop(0.0),
        new ShooterOpenLoop(0.0)
      ),
      // new DriveTrajectory(AutoTrajectories.midCargoToAllianceWall)
      new DriveTrajectory(AutoTrajectories.midCargoToHumanPlayer)
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
