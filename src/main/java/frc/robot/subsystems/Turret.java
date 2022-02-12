package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Test;

public class Turret extends ServoMotorSubsystem {
  public Turret() {
    super(Constants.Turret.SERVO);
    // TODO uncomment after 4-pin JST connector is wired
    // TalonUtil.checkError(mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), getName() + ": Could not set forward limit switch: ");
    // TalonUtil.checkError(mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), getName() + ": Could not set reverse limit switch: ");
    // mMaster.overrideLimitSwitchesEnable(true);
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
    SmartDashboard.putBoolean("Turret Limit Switch F", mMaster.getSensorCollection().isFwdLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("Turret Limit Switch R", mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1);
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
    Test.checkFirmware(new Test.FirmwareTalon(this, mMaster));
  }
}
