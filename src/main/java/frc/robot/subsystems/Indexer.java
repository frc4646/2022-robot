package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
public class Indexer extends SubsystemBase {
  // TODO motor

  
    
  private final VictorSPX motor;

  public Indexer() {
    motor = new VictorSPX(Constants.Ports.INDEXER);
    // TODO supply current limiting
  }

  public void setIndex (double index) {
    motor.set(ControlMode.PercentOutput, index);
  }


  public void setOpenLoop(double percent) {
    // TODO motor percent output mode
  }
}
