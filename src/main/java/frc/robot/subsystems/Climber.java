package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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
  private boolean extended = false;

  public Climber() {
    // masterL = TalonFXFactory.createDefaultTalon(Constants.CAN.CLIMBER_L);
    // masterR = TalonFXFactory.createDefaultTalon(Constants.CAN.CLIMBER_R);
    cylinderL = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.CLIMBER_L_OUT, Constants.Solenoid.CLIMBER_L_IN);
    cylinderR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.CLIMBER_R_OUT, Constants.Solenoid.CLIMBER_R_IN);

    // masterL.changeMotionControlFramePeriod(60);
    // masterL.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 60, 100);

    // masterL.set(ControlMode.PercentOutput, 0);
    // masterL.setInverted(true);
    // masterL.configVoltageCompSaturation(12.0, Constants.CAN_TIMEOUT_LONG);
    // masterL.enableVoltageCompensation(true);

    // masterL.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.CAN_TIMEOUT_LONG);

    // masterL.configMotionAcceleration(40000, Constants.CAN_TIMEOUT_LONG);
    // masterL.configMotionCruiseVelocity(30000, Constants.CAN_TIMEOUT_LONG);
    // masterL.config_kP(0, 0.5);
    // masterL.config_kI(0, 0);
    // masterL.config_kD(0, 0);
    // masterL.config_kF(0, 0.05);

    // masterL.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT_LONG);

    // masterL.setNeutralMode(NeutralMode.Coast);

    // StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 80, 80, 1.0);
    // masterL.configStatorCurrentLimit(STATOR_CURRENT_LIMIT);

    isBrakeMode = false;
    setBrakeMode(true);
  }

  @Override
  public void updateDashboard() {    
    //SmartDashboard.putBoolean("Climber Limit L", masterL.getSensorCollection().isRevLimitSwitchClosed() == 1);
    //SmartDashboard.putBoolean("Climber Limit R", masterR.getSensorCollection().isRevLimitSwitchClosed() == 1);
  }

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode == enable) {
      return;  // Already in this mode
    }
    NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
    // masterL.setNeutralMode(mode);
    // masterR.setNeutralMode(mode);
    isBrakeMode = enable;
  }

  public void setOpenLoop(double percent) {
    // masterL.set(TalonFXControlMode.PercentOutput, percent);
  }

  public void setArms(boolean extend) {
    Value direction = (extend) ? Value.kForward : Value.kReverse;
    cylinderL.set(direction);
    cylinderR.set(direction);
    extended = extend;
  }

  public void setRatchet(boolean enable) {
    // TODO
  }

  public boolean isArmsExtended() {
    return extended;
  }

  public boolean isStalled() {
    return false;  // TODO zero if stalled at bottom rather than snap cable
  }

  @Override
  public void runTests() {
    // Test.checkFirmware(new Test.FirmwareSparkMax(this, masterL));
    // Test.checkFirmware(new Test.FirmwareSparkMax(this, masterR));
    Test.checkSolenoid(this, cylinderL);
    Test.checkSolenoid(this, cylinderR);
  }
}
