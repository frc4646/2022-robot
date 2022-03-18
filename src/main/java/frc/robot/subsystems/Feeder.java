package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.Test;

public class Feeder extends SmartSubsystem {
  private class DataCache {
    public boolean shooterLoaded, shooterLoadedBottom;
    public double position;
  }

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pid;
  private final DigitalInput breakBeam, breakBeamBottom;
  private final DataCache cache = new DataCache();
  private double demand = 0.0;
  private boolean isBrakeMode;

  public Feeder() {
    motor = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.FEEDER);
    breakBeam = new DigitalInput(Constants.DIGITAL.FEEDER_BREAK_BEAM);
    breakBeamBottom = new DigitalInput(Constants.DIGITAL.FEEDER_BOTTOM_BREAK_BEAM);
    encoder = motor.getEncoder();
    pid = motor.getPIDController();

    motor.setInverted(true);
    motor.enableVoltageCompensation(12.0);
    motor.setOpenLoopRampRate(Constants.FEEDER.OPEN_LOOP_RAMP);
    pid.setP(Constants.FEEDER.P);
    pid.setI(Constants.FEEDER.I);
    pid.setD(Constants.FEEDER.D);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    isBrakeMode = true;
    setBrakeMode(false);
  }

  @Override
  public void cacheSensors () {
    cache.shooterLoaded = !breakBeam.get();
    cache.shooterLoadedBottom = !breakBeamBottom.get();
    cache.position = encoder.getPosition();
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    SmartDashboard.putBoolean("Feeder: Loaded", isShooterLoaded());
    SmartDashboard.putBoolean("Feeder: Bottom Loaded", isShooterLoadedBottom());
    if (Constants.TUNING.FEEDER) {
      SmartDashboard.putNumber("Feeder: Position", getPosition());
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

  public void setOpenLoop(double percent) {
    motor.set(percent);
    demand = percent;
  }

  public void setClosedLoopPosition(double position) {
    double setpoint = position * Constants.FEEDER.GEAR_RATIO;
    pid.setReference(setpoint, ControlType.kPosition);
    demand = setpoint;
  }  

  public void setBrakeMode(boolean enable) {
    if (isBrakeMode == enable) {
      return;
    }
    IdleMode mode = enable ? IdleMode.kBrake : IdleMode.kCoast;
    motor.setIdleMode(mode);
    isBrakeMode = enable;
  }

  public double getPosition() { return cache.position / Constants.FEEDER.GEAR_RATIO; }

  public int getQueuedCargo() {
    return (isShooterLoaded() ? 1 : 0) +
           (isShooterLoadedBottom() ? 1 : 0);  // TODO
  }

  public int getQueuedCargoCorrect() {
    return 0;  // TODO
  }

  public int getQueuedCargoWrong() {
    return 0;  // TODO
  }

  public boolean isOnTarget() { return Math.abs(getPosition() - demand) < Constants.FEEDER.POSITION_DEADBAND; }
  public boolean isShooterLoaded() { return cache.shooterLoaded; }
  public boolean isShooterLoadedBottom() { return cache.shooterLoadedBottom; }

  @Override
  public void runTests() {
    Test.checkFirmware(this, motor);
  }
}
