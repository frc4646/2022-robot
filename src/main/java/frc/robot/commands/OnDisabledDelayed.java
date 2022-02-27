package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class OnDisabledDelayed extends CommandBase {  
  private final double TIME_ON_DISABLE_DELAYED = Constants.Field.CLIMBER_TIME_REQUIRED_TO_HOLD * 2.0;
  private final Climber climber = RobotContainer.CLIMBER;
  private final Drivetrain drive = RobotContainer.DRIVETRAIN;
  private double timeStarted = Double.MAX_VALUE;

  public OnDisabledDelayed() {
    //addRequirements(climber);
  }

  @Override
  public void initialize() {
    timeStarted = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - timeStarted > TIME_ON_DISABLE_DELAYED;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Setting coast!");
    // TODO climber.setBrakeMode(false);  // Delay so 5 second hold requirement met in matches
    drive.setBrakeMode(false);  // Delay so inertia cancels before allowing coast
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
