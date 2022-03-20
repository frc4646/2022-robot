package frc.team4646;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.TalonUtil;

/** Updates PIDF of motor controller without needing to program */
public class PIDTuner {
  private static final String DASHBOARD_KEY_P = "Tuner: P";
  private static final String DASHBOARD_KEY_I = "Tuner: I";
  private static final String DASHBOARD_KEY_D = "Tuner: D";
  private static final String DASHBOARD_KEY_F = "Tuner: F";
  private static final String DASHBOARD_KEY_CRACKPOINT = "Tuner: CRACKPOINT";
  private final double DEFAULT_P;
  private final double DEFAULT_I;
  private final double DEFAULT_D;
  private final double DEFAULT_F;
  private final double DEFAULT_CRACKPOINT;
  private final int SLOT;
  private final List<TalonFX> motors;

  /** Updates PIDF of motor controller without needing to program */
  public PIDTuner(TalonFX... motors) {
    this(0.0, 0.0, 0.0, 0.0, 0.0, 0, motors);
  }

  /** Updates PIDF of motor controller without needing to program */
  public PIDTuner(double P, double I, double D, double F, double crackpoint, TalonFX... motors) {
    this(P, I, D, F, 0.0, 0, motors);
  }

  /** Updates PIDF of motor controller without needing to program */
  public PIDTuner(double P, double I, double D, double F, double crackpoint, int slot, TalonFX... motors) {
    this.motors = List.of(motors);
    DEFAULT_P = P;
    DEFAULT_I = I;
    DEFAULT_D = D;
    DEFAULT_F = F;
    DEFAULT_CRACKPOINT = crackpoint;
    SLOT = slot;
    SmartDashboard.putNumber(DASHBOARD_KEY_P, DEFAULT_P);
    SmartDashboard.putNumber(DASHBOARD_KEY_I, DEFAULT_I);
    SmartDashboard.putNumber(DASHBOARD_KEY_D, DEFAULT_D);
    SmartDashboard.putNumber(DASHBOARD_KEY_F, DEFAULT_F);
    SmartDashboard.putNumber(DASHBOARD_KEY_CRACKPOINT, DEFAULT_CRACKPOINT);
  }

  /** 
   * Call to refresh motor controller PIDF with values from Smart Dashboard.
   * Suggest from subsystem.OnEnabled() or subsystem.OnDisabled().
   */
  public void updateMotorPIDF() {
    double P = SmartDashboard.getNumber(DASHBOARD_KEY_P, 0.0);
    double I = SmartDashboard.getNumber(DASHBOARD_KEY_I, 0.0);
    double D = SmartDashboard.getNumber(DASHBOARD_KEY_D, 0.0);
    double F = SmartDashboard.getNumber(DASHBOARD_KEY_F, 0.0);
    
    for (TalonFX motor : motors) {
      TalonUtil.checkError(motor.config_kP(SLOT, P, Constants.CAN_TIMEOUT), "Tuner: could not set P: ");
      TalonUtil.checkError(motor.config_kI(SLOT, I, Constants.CAN_TIMEOUT), "Tuner: could not set I: ");
      TalonUtil.checkError(motor.config_kD(SLOT, D, Constants.CAN_TIMEOUT), "Tuner: could not set D: ");
      TalonUtil.checkError(motor.config_kF(SLOT, F, Constants.CAN_TIMEOUT), "Tuner: could not set F: ");
    }
  }
  
  /** @return crackpoint. Pass into motor controller's set function. */
  public double getCrackpoint() {
    return SmartDashboard.getNumber(DASHBOARD_KEY_CRACKPOINT, 0.0);
  }
}
