package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.team254.drivers.TalonUtil;
import frc.team4646.StabilityCounter;
import frc.team4646.Test;

public class Turret extends ServoMotorSubsystem {
  private class DataCache {
    public boolean limitF, limitR;
  }

  private final Canifier canifier;
  private final StabilityCounter stability = new StabilityCounter(Constants.TURRET.STABLE_COUNTS);
  private final DataCache cache = new DataCache();
  
  public Turret() {
    super(Constants.TURRET.SERVO);
    canifier = RobotContainer.CANIFIER;
    TalonUtil.checkError(mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), "Turret: Could not set forward limit switch: ");
    TalonUtil.checkError(mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), "Turret: Could not set reverse limit switch: ");
    mMaster.overrideLimitSwitchesEnable(true);
    mMaster.overrideSoftLimitsEnable(true);
    forceZero();
    setBrakeMode(false);
  }

  @Override
  public void cacheSensors() {
    super.cacheSensors();
    cache.limitF = mMaster.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    cache.limitR = mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1;
    resetIfAtHome();
    stability.calculate(Math.abs(mPeriodicIO.error_ticks) <= Constants.TURRET.SERVO.kMotionMagicDeadband);
    // TODO use position to compute error
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    super.updateDashboard(showDetails);
    if (Constants.TUNING.TURRET) {
      SmartDashboard.putBoolean("Turret: Limit F", cache.limitF);
      SmartDashboard.putBoolean("Turret: Limit R", cache.limitR);
      SmartDashboard.putNumber("Turret: Error", mPeriodicIO.error_ticks);
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    setBrakeMode(true);
    setOpenLoop(0.0);  // Handle if zeroed while disabled
    mMaster.set(TalonFXControlMode.PercentOutput, 0.0);  // Send a neutral command
  }

  @Override
  public void onDisable() {
    setBrakeMode(false);
  }

  @Override
  public void resetIfAtHome() {
    if (atHomingLocation()) {
      if (!mHasBeenZeroed) {
        mMaster.overrideSoftLimitsEnable(true);
      }
      zeroSensors();
    }
  }

  @Override
  public boolean atHomingLocation() {
    return canifier.isTurretHome();
  }

  public boolean isOnTarget() {
    return stability.isStable();
  }

  public double wrapIfPastDeadzone(double degrees) {
    double limitMin = Constants.TURRET.SERVO.kMinUnitsLimit;
    double limitMax = Constants.TURRET.SERVO.kMaxUnitsLimit;
    double aboveMax = degrees - limitMax;
    double belowMin = limitMin - degrees;

    if (aboveMax > 0.0) {
      return limitMin + aboveMax;  // Ex: want 180 above max, use 20 above min instead
    } else if (belowMin < 0.0) {
      return limitMax + belowMin;
    }
    return degrees;
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, mMaster);
    Test.checkStatusFrames(this, mMaster);
  }
}
