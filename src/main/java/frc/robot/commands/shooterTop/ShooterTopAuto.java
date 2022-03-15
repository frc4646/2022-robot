package frc.robot.commands.shooterTop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterTop;

public class ShooterTopAuto extends CommandBase {
  private final ShooterTop subsystem = RobotContainer.SHOOTER_TOP;

  public ShooterTopAuto() {
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double setpoint = 0.0;
    subsystem.setClosedLoop(setpoint);
  }
}
