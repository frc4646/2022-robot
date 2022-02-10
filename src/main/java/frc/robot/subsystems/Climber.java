package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;
import frc.robot.util.Test;

public class Climber extends SmartSubsystem {
  // private final TalonFX masterL, masterR;
  private final DoubleSolenoid cylinderL, cylinderR;

  private boolean isBrakeMode;

  public Climber() {
    // masterL = TalonFXFactory.createDefaultTalon(Constants.CAN.CLIMBER_L);
    // masterR = TalonFXFactory.createDefaultTalon(Constants.CAN.CLIMBER_R);
    cylinderL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.CLIMBER_L_OUT, Constants.Solenoid.CLIMBER_L_IN);
    cylinderR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.CLIMBER_R_OUT, Constants.Solenoid.CLIMBER_R_IN);

    // TODO configure motors

    isBrakeMode = false;
    setBrakeMode(true);
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode != enable) {
      NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
      // masterL.setNeutralMode(mode);
      // masterR.setNeutralMode(mode);
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
