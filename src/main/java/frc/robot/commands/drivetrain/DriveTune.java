package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveTune extends CommandBase {
  private static final String DASHBOARD_KEY_SETPOINT = "Tune: Drive Setpoint";
  private static final String DASHBOARD_KEY_MODE = "Tune: Drive Closed Loop";
  private final Drivetrain subsystem = RobotContainer.DRIVETRAIN;

  public DriveTune() {
    addRequirements(subsystem);
    SmartDashboard.putNumber(DASHBOARD_KEY_SETPOINT, 0.0);
    SmartDashboard.putNumber(DASHBOARD_KEY_MODE, 0);
  }

  @Override
  public void initialize() {
    subsystem.setBrakeMode(true);
    double setpoint = SmartDashboard.getNumber(DASHBOARD_KEY_SETPOINT, 0.0);
    boolean closedLoop = SmartDashboard.getBoolean(DASHBOARD_KEY_MODE, true);
    if (closedLoop) {
      subsystem.setClosedLoopVelocity(setpoint, setpoint);
    } else {
      subsystem.setOpenLoop(setpoint, setpoint);
    }
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setBrakeMode(true);
    subsystem.setOpenLoop(0.0, 0.0);
  }
}
