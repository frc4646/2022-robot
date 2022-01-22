package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  // TODO motor(s)
  private final CANSparkMax leftMaster,rightMaster;

  public Shooter() {
    // TODO configure motors 
    
    // TODO limit supply current
    leftMaster = new CANSparkMax(Constants.Ports.SHOOTER_L, MotorType.kBrushless);
    rightMaster = new CANSparkMax(Constants.Ports.SHOOTER_R, MotorType.kBrushless);
    // TODO PIDF
    // TODO peak output forward direction only
    // TODO coast mode
  }

  public void setOpenLoop(double percent) {
    // TODO set percent voltage mode
  }

  public void setSpeed(double velocity) {
    leftMaster.set(velocity);
    rightMaster.set(velocity);
    // TODO set velocity control mode
  }

  public int getSpeed() {
    return 0;  // TODO read sensor
  }

  public boolean isOnTarget() {
    // TODO is close enough
    // TODO increment how many times is stable    
    return false;  // TODO decide is enough times is stable
  }
}
