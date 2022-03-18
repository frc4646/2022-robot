package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveTune extends CommandBase {
  private final Drivetrain subsystem = RobotContainer.DRIVETRAIN;
  private final double setpoint;

  public DriveTune(double metersPerSecond) {
    addRequirements(subsystem);
    setpoint = metersPerSecond;
  }

  @Override
  public void initialize() {
    subsystem.setBrakeMode(false);
    subsystem.setClosedLoopVelocity(setpoint, setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.setBrakeMode(true);
    subsystem.setOpenLoop(0.0, 0.0);
  }
}
