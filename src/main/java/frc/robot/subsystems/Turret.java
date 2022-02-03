package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Turret extends SmartSubsystem {
  public static class DataCache {
    public double amps;
  }

  private final CANSparkMax motor;
  // private final RelativeEncoder encoder;
  private final DataCache cache = new DataCache();

  public Turret() {
    motor = new CANSparkMax(Constants.Ports.TURRET, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.getPIDController().setP(Constants.Turret.P);
    motor.getPIDController().setI(Constants.Turret.I);
    motor.getPIDController().setD(Constants.Turret.D);
    motor.getPIDController().setFF(Constants.Turret.F);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);  // Faster position feedback

    // encoder = motor.getAlternateEncoder(1024);  TODO should we use getEncoder or getAlternateEncoder???
    // TODO see https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Alternate%20Encoder/src/main/java/frc/robot/Robot.java
    // TODO see https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java
  }

  @Override
  public void cacheSensors() {
    cache.amps = motor.getOutputCurrent();
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber(this.getName() + " Amps", cache.amps);
  }

  public void resetEncoders() {
    // TODO
  }

  public void setOpenLoop(double percent) {
    motor.set(percent);
  }

  public double getPositionRaw() {
    return 0.0;  // TODO
  }
}
