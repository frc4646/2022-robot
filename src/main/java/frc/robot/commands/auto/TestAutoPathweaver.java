package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.drivetrain.DrivePath;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.feeder.WaitForColorState;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.shooter.ShooterOpenLoop;
import frc.robot.subsystems.ColorSensor.STATE;

public class TestAutoPathweaver extends ModeBase {

  public TestAutoPathweaver() {
    addCommands(
      new InstantCommand(() -> { RobotContainer.DRIVETRAIN.resetPose(GeneratedPaths.MidStart_MidCargo.getInitialPose()); }),   
      new DeployIntake(),
      new WaitCommand(1.0),
      new DrivePath(GeneratedPaths.MidStart_MidCargo),
      new WaitCommand(.2),
      new WaitForColorState(STATE.CORRECT).withTimeout(2.0),
      new ShootVision(),
      new AgitateOpenLoop(0.0),
      new FeederOpenLoop(0.0),
      new ShooterOpenLoop(0.0),
      new DrivePath(GeneratedPaths.MidCargo_Terminal)
    );
  }
}
