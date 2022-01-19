package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterTune extends CommandBase {
  private Shooter subsystem = RobotContainer.SHOOTER;

  public ShooterTune() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    int setpoint = (int) SmartDashboard.getNumber("tuning/shooterVel", 0);
    subsystem.setOpenLoop(setpoint);  // TODO use velocity mode once we have an encoder
    //subsystem.setSpeed(setpoint);
  }
}
