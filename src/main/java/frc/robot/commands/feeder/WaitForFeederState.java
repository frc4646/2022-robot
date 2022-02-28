package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class WaitForFeederState extends CommandBase {
  private final Feeder subsystem = RobotContainer.FEEDER;
  private final boolean waitingForIsLoaded;

  public WaitForFeederState(boolean waitForIsLoaded) {
    waitingForIsLoaded = waitForIsLoaded;
  }

  @Override
  public boolean isFinished() {
    return waitingForIsLoaded ? subsystem.isShooterLoaded() : !subsystem.isShooterLoaded();
  }
}
