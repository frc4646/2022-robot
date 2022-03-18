package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.feeder.FeederLoadCargo;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.shooter.WaitForShooterVelocity;

public class SmartFireCargo extends SequentialCommandGroup {
  public SmartFireCargo() {
    addCommands(
      new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_SHOOT),
      new WaitCommand(0.25),
      new FeederLoadCargo().withTimeout(1.0),
      new ConditionalCommand( // shoot second if we have it
        sequence(
          new WaitForShooterVelocity(),
          new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_SHOOT),
          new WaitCommand(0.25),
          new FeederLoadCargo().withTimeout(1.0),
          new ConditionalCommand( // shoot third if we have it
            sequence(
              new WaitForShooterVelocity(),
              new FeederOpenLoop(Constants.FEEDER.OPEN_LOOP_SHOOT),
              new WaitCommand(0.5)
            ),
            new InstantCommand(() -> {}),
            RobotContainer.FEEDER.isShooterLoaded
          )
        ),
        new InstantCommand(() -> {}),
        RobotContainer.FEEDER.isShooterLoaded
      )
    );
  }
}
