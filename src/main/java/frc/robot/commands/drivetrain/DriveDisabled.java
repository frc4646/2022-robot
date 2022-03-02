package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveDisabled extends CommandBase {
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private double timeStarted = Double.MAX_VALUE;

  @Override
  public void initialize() {
    timeStarted = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - timeStarted > Constants.Drivetrain.TIMEOUT_DISABLED_COAST;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Setting coast!");
    drive.setBrakeMode(false);  // Delay so inertia cancels before allowing coast
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
