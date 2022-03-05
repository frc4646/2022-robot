package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.Test;
import frc.team254.drivers.TalonUtil;

public class Turret extends ServoMotorSubsystem {
  private static class DataCache {
    public boolean limitF, limitR;
  }

  private final Canifier canifier;
  private final DataCache cache = new DataCache();
  
  public Turret() {
    super(Constants.TURRET.SERVO);
    canifier = RobotContainer.CANIFIER;
    TalonUtil.checkError(mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), getName() + ": Could not set forward limit switch: ");
    TalonUtil.checkError(mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), getName() + ": Could not set reverse limit switch: ");
    mMaster.overrideLimitSwitchesEnable(true);
    mMaster.overrideSoftLimitsEnable(true);
    forceZero();
  }

  @Override
  public void cacheSensors() {
    super.cacheSensors();
    cache.limitF = mMaster.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    cache.limitR = mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1;
    if (atHomingLocation()) {
      zeroSensors();
    }
  }

  @Override
  public void updateDashboard() {
    super.updateDashboard();
    SmartDashboard.putBoolean("Turret: Limit F", cache.limitF);
    SmartDashboard.putBoolean("Turret: Limit R", cache.limitR);
    if (Constants.TURRET.TUNING) {
      SmartDashboard.putNumber("Turret: Error", mPeriodicIO.error_ticks);
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    setBrakeMode(true);
    setSetpointPositionPID(getPosition(), 0.0);  // Handle if zeroed while disabled
    // setOpenLoop(0.0);  // TODO try if better
  }

  @Override
  public void onDisable() {
    setBrakeMode(false);
  }

  @Override
  public boolean atHomingLocation() {
    return canifier.isTurretHome();
  }

  public boolean isOnTarget() {
    return true; // TODO Math.abs(mPeriodicIO.error_ticks) < 1.0 * Constants.Turret.SERVO.kTicksPerUnitDistance;
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, mMaster);
    Test.checkStatusFrames(this, mMaster);
  }
}
