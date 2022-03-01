package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.drivers.TalonFXFactory;
import frc.team254.drivers.TalonUtil;

public class ClimberArms extends SmartSubsystem {

  private final DoubleSolenoid armL, armR;//, ratchetL, ratchetR;

  private boolean armsExtended = false;

  public ClimberArms() {
    armL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.ARM_L_OUT, Constants.Solenoid.ARM_L_IN);
    armR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.ARM_R_OUT, Constants.Solenoid.ARM_R_IN);
    // ratchetL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.RATCHET_L_OUT, Constants.Solenoid.RATCHET_L_IN);
    // ratchetR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.RATCHET_R_OUT, Constants.Solenoid.RATCHET_R_IN);

    setArms(false);  // solenoid default is OFF, not IN
  }

  @Override
  public void cacheSensors() {
  }

  @Override
  public void updateDashboard() {
  }

  @Override
  public void onEnable(boolean isAutonomous) {
  }
  public void setArms(boolean extend) {
    Value direction = (extend) ? Value.kForward : Value.kReverse;
    armL.set(direction);
    armR.set(direction);
    armsExtended = extend;
  }

  public boolean isArmsExtended() {
    return armsExtended;
  }

  @Override
  public void runTests() {
    Test.checkSolenoid(this, armL);
    Test.checkSolenoid(this, armR);
    // Test.checkSolenoid(this, ratchetL);
    // Test.checkSolenoid(this, ratchetR);
  }
}
