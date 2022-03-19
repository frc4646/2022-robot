package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ResetAuto extends CommandBase {
  private static String DASHBOARD_KEY_DELAY = "Auto Delay";
  private final Pose2d initialPosition;
  private double delay = 0.0, timeStarted = 0.0;
  
  public ResetAuto() {
    this(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  }

  public ResetAuto(Pose2d initialPosition) {
    SmartDashboard.putNumber(DASHBOARD_KEY_DELAY, 0.0);
    this.initialPosition = initialPosition;
  }

  @Override
  public void initialize() {
    timeStarted = Timer.getFPGATimestamp();
    delay = SmartDashboard.getNumber(DASHBOARD_KEY_DELAY, 0.0);
    if (initialPosition != null) {
      RobotContainer.DRIVETRAIN.resetPose(initialPosition);
    }
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - timeStarted > delay;
  }
}
