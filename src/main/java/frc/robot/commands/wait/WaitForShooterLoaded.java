package frc.robot.commands.wait;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class WaitForShooterLoaded extends CommandBase {
  private final Feeder subsystem = RobotContainer.FEEDER;
  private final boolean waitingForIsLoaded;

  public WaitForShooterLoaded(boolean waitForIsLoaded) {
    waitingForIsLoaded = waitForIsLoaded;
  }

  @Override
  public boolean isFinished() {
    return waitingForIsLoaded ? subsystem.isShooterLoaded() : !subsystem.isShooterLoaded();
  }
}
