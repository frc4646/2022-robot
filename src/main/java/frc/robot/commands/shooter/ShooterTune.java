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
    double setpoint = (double) SmartDashboard.getNumber("Tune Shooter", 1);
    subsystem.setOpenLoop(setpoint);  // TODO use velocity mode once we have an encoder
    //subsystem.setSpeed(setpoint);

    System.out.println("Making shot,"
      + " range: " + vision.getGroundDistanceToHubInches()
      + " rpm: " + subsystem.getRPM()
      + " volts: " + subsystem.getVoltage()
      + " amps supply: " + subsystem.getAmpsSupply()
      + " amps stator: " + subsystem.getAmpsStator());
  }
}
