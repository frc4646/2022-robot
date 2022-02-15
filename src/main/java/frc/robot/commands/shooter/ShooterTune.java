package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class ShooterTune extends CommandBase {
  private Shooter subsystem = RobotContainer.SHOOTER;
  private Vision vision = RobotContainer.VISION;

  public ShooterTune() {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    double setpoint = SmartDashboard.getNumber("Tune: Setpoint", 0.0);
    subsystem.setClosedLoop(setpoint);
  }

  @Override
  public boolean isFinished() {
    return subsystem.isStable();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Tune,"
    + " range: " + vision.getGroundDistanceToHubInches()
    + " rpm: " + subsystem.getRPM()
    + " volts: " + subsystem.getVoltage()
    + " amps supply: " + subsystem.getAmpsSupply()
    + " amps stator: " + subsystem.getAmpsStator());
  }
}
