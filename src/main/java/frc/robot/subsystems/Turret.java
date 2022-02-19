package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;
import frc.team254.drivers.TalonUtil;

public class Turret extends ServoMotorSubsystem {
  public Turret() {
    super(Constants.Turret.SERVO);
    TalonUtil.checkError(mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), getName() + ": Could not set forward limit switch: ");
    TalonUtil.checkError(mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), getName() + ": Could not set reverse limit switch: ");
    mMaster.overrideLimitSwitchesEnable(true);
    mMaster.overrideSoftLimitsEnable(true);
    forceZero();
  }

  @Override
  public void cacheSensors() {
    super.cacheSensors();
    if (hasBeenZeroed()) {
      // TODO mLED.clearTurretFault();
    }
  }

  @Override
  public void updateDashboard() {
    super.updateDashboard();
    SmartDashboard.putBoolean("Turret: Limit F", mMaster.getSensorCollection().isFwdLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("Turret: Limit R", mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1);
    if (Constants.Turret.TUNING) {
      SmartDashboard.putNumber("Turret: Error", mPeriodicIO.error_ticks);
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    setBrakeMode(true);
  }

  @Override
  public void onDisable() {
    setBrakeMode(false);
  }

  @Override
  public boolean atHomingLocation() {
    return false;  // TODO canifier.isTurretHomed();
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, mMaster);
  }
}
