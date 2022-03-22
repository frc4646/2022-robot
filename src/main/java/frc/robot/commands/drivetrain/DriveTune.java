package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveTune extends SequentialCommandGroup {
  private static final String DASHBOARD_KEY_DURATION = "Tune: Drive Seconds";

  public DriveTune() {
    addCommands(
      new DriveTest(),
      new DriveTest(.9).withTimeout(0.1),
      new DriveTest(.8).withTimeout(0.1),
      new DriveTest(.7).withTimeout(0.1),
      new DriveTest(.6).withTimeout(0.1),
      new DriveTest(.5).withTimeout(0.1),
      new DriveTest(.4).withTimeout(0.1),
      new DriveTest(.3).withTimeout(0.1),
      new DriveTest(.2).withTimeout(0.1),
      new DriveTest(.1).withTimeout(0.1),
      new DriveOpenLoop()
    );
  }
  private class DriveTest extends CommandBase {
  private static final String DASHBOARD_KEY_SETPOINT = "Tune: Drive Setpoint";
    private static final String DASHBOARD_KEY_MODE = "Tune: Drive Closed Loop";
    private final Drivetrain subsystem = RobotContainer.DRIVETRAIN;
    private final double scale;
    private double driveSeconds;
    private double startTime = Double.MIN_VALUE;

    public DriveTest(double scaleSetpoint) {
      addRequirements(subsystem);
      SmartDashboard.putNumber(DASHBOARD_KEY_SETPOINT, 0.75);
      SmartDashboard.putBoolean(DASHBOARD_KEY_MODE, true);
      SmartDashboard.putNumber(DASHBOARD_KEY_DURATION, 1.0);
      scale = scaleSetpoint;
    }

    public DriveTest() {
      this(1.0);
    }

    @Override
    public void initialize() {
      startTime = Timer.getFPGATimestamp();
      driveSeconds = SmartDashboard.getNumber(DASHBOARD_KEY_DURATION, 0.0);
      subsystem.setBrakeMode(false);
      double setpoint = SmartDashboard.getNumber(DASHBOARD_KEY_SETPOINT, 0.0) * scale;
      boolean closedLoop = SmartDashboard.getBoolean(DASHBOARD_KEY_MODE, true);      
      if (closedLoop) {
        subsystem.setClosedLoopVelocity(setpoint, setpoint);
      } else {
        subsystem.setOpenLoop(setpoint, setpoint);
      }
    }

    @Override
    public boolean isFinished() {
      return Timer.getFPGATimestamp() - startTime >= driveSeconds;
    }

    @Override
    public void end(boolean interrupted) {
      subsystem.setOpenLoop(0.0, 0.0);
    }
  }
}
