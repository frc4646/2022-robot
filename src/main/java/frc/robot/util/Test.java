package frc.robot.util;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Test {
  private static class Firmware {
    public final String subsystem;
    public final int deviceID;
    public final int actual;
    public final int expected;
    
    protected Firmware(SubsystemBase subsystem, int deviceID, int actual, int expected) {
      this.subsystem = subsystem.getName();
      this.deviceID = deviceID;
      this.actual = actual;
      this.expected = expected;
    }
  }

  public static class FirmwareTalon extends Firmware {
    public FirmwareTalon(SubsystemBase subsystem, BaseMotorController device) {
      super(subsystem, device.getDeviceID(), device.getFirmwareVersion(), 0x2200);
    }
  }

  public static class FirmwareSparkMax extends Firmware {
    public FirmwareSparkMax(SubsystemBase subsystem, CANSparkMax device) {
      super(subsystem, device.getDeviceId(), device.getFirmwareVersion(), 0x01050200);
    }
  }

  /** Check if motor controllers need a software update */
  public static boolean checkFirmware(Firmware firmware) {
    boolean result = firmware.actual == firmware.expected;
    System.out.println(String.format("%s device %d firmware %s, expected %s: %s", firmware.subsystem, firmware.deviceID, firmware.actual, firmware.expected, getResultString(result)));
    return result;
  }

  /** Check if pneumantic control module detects wires are short. Power cycle robot when shorted. */
  public static boolean checkSolenoid(SubsystemBase subsystem, DoubleSolenoid solenoid) {
    boolean result = !solenoid.isFwdSolenoidDisabled();
    System.out.println(String.format("%s solenoid %d/%d: %s", subsystem.getName(), solenoid.getFwdChannel(), solenoid.getRevChannel(), getResultString(result)));
    return result;
  }

  public static String getResultString(boolean result) {
    return (result) ? "Okay" : "ERROR";
  }
}
