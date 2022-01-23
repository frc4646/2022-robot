package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
public class Indexer extends SubsystemBase {
    
  private final VictorSPX motor;

  public Indexer() {
    motor = new VictorSPX(Constants.Ports.INDEXER);
    // TODO supply current limiting
    // TODO add sensor for if ball is in indexer
  }

  public void setOpenLoop(double percent) {
    motor.set(ControlMode.PercentOutput, percent);
  }
}
