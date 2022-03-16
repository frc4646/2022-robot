package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.team254.drivers.SparkMaxFactory;
import frc.team4646.Test;

public class Feeder extends SmartSubsystem {
  private static class DataCache {
    public boolean shooterLoaded;
    public double position;
  }

  private final CANSparkMax motor;
  private final DigitalInput breakBeam;
  private DataCache cache = new DataCache();
  private double demand = 0.0;

  public BooleanSupplier isShooterLoaded = () -> isShooterLoaded();

  public Feeder() {
    motor = SparkMaxFactory.createDefaultSparkMax(Constants.CAN.FEEDER);
    breakBeam = new DigitalInput(Constants.DIGITAL.FEEDER_BREAK_BEAM);

    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kBrake);
    motor.enableVoltageCompensation(12.0);
    motor.setOpenLoopRampRate(Constants.FEEDER.OPEN_LOOP_RAMP);
    motor.getPIDController().setP(Constants.FEEDER.P);
    motor.getPIDController().setI(Constants.FEEDER.I);
    motor.getPIDController().setD(Constants.FEEDER.D);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
  }

  @Override
  public void cacheSensors () {
    cache.shooterLoaded = !breakBeam.get();
    cache.position = motor.getEncoder().getPosition();
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    SmartDashboard.putBoolean("Feeder: Loaded", isShooterLoaded());
    if (Constants.FEEDER.TUNING) {
      SmartDashboard.putNumber("Feeder: Position", getPosition());      
    }
  }

  public void setOpenLoop(double percent) {
    motor.set(percent);
    demand = percent;
  }

  public void setClosedLoopPosition(double position) {
    double setpoint = position * Constants.FEEDER.GEAR_RATIO;
    motor.getPIDController().setReference(setpoint, ControlType.kPosition);
    demand = setpoint;
  }

  public double getPosition() {
    return cache.position / Constants.FEEDER.GEAR_RATIO;
  }

  public int getQueuedCargo() {
    return 0;  // TODO
  }

  public int getQueuedCargoCorrect() {
    return 0;  // TODO
  }

  public int getQueuedCargoWrong() {
    return 0;  // TODO
  }

  public boolean isOnTarget() {
    return Math.abs(getPosition() - demand) < Constants.FEEDER.POSITION_DEADBAND;
  }

  public boolean isShooterLoaded() {
    return cache.shooterLoaded;
  }

  @Override
  public void runTests() {
    Test.checkFirmware(this, motor);
  }
}
