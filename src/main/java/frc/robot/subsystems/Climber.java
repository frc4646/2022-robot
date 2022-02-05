package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.util.Test;

public class Climber extends SmartSubsystem {
  // TODO motors
  private final DoubleSolenoid cylinderL, cylinderR;

  private boolean isBrakeMode;

  public Climber() {
    // TODO configure motors
    cylinderL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.CLIMBER_L_OUT, Constants.Solenoid.CLIMBER_L_IN);
    cylinderR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.CLIMBER_R_OUT, Constants.Solenoid.CLIMBER_R_IN);

    isBrakeMode = false;
    setBrakeMode(true);
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode != enable) {
      IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
      // TODO set mode on motors
      isBrakeMode = enable;
    }
  }

  public void setOpenLoop(double percent) {
    // TODO
  }

  public void setArms(boolean extend) {
    Value direction = (extend) ? Value.kForward : Value.kReverse;
    cylinderL.set(direction);
    cylinderR.set(direction);
  }

  @Override
  public void runTests() {
    // TODO motor firmware
    Test.checkSolenoid(this, cylinderL);
    Test.checkSolenoid(this, cylinderR);
  }
}
