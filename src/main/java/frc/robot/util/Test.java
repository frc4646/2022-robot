package frc.robot.util;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Test report system */
public class Test {
  private static int numPass = 0, numFail = 0;

  /** Include in test report */
  public static void add(String testCriteria, boolean result) {
    if (result) {
      numPass++;
    } else {
      numFail++;
    }
    String sResult = (result) ? "Okay   " : "ERROR  ";
    System.out.println(sResult + testCriteria);
  }

  /** Zero test report metrics */
  public static void reset() {
    banner();
    numPass = 0;
    numFail = 0;
  }
  
  /** Summarize test report */
  public static boolean results() {
    System.out.println("Pass: " + numPass + " Tests");
    System.out.println("Fail: " + numFail + " Tests");
    banner();
    return numFail == 0;
  }

  /** Check if motor controllers need a software update */
  public static void checkFirmware(Firmware firmware) {
    boolean result = firmware.actual == firmware.expected;
    add(String.format("%s Device %d Firmware 0x%X, Expected 0x%X", firmware.subsystem, firmware.deviceID, firmware.actual, firmware.expected), result);
  }

  /** Check if pneumantic control module detects wires are short. Power cycle robot when shorted. */
  public static void checkSolenoid(SubsystemBase subsystem, DoubleSolenoid solenoid) {
    boolean result = !solenoid.isFwdSolenoidDisabled();
    add(String.format("%s Solenoid %d/%d", subsystem.getName(), solenoid.getFwdChannel(), solenoid.getRevChannel()), result);
  }

  public static class FirmwareTalon extends Firmware {
    public FirmwareTalon(SubsystemBase subsystem, BaseMotorController device) {
      super(subsystem, device.getDeviceID(), device.getFirmwareVersion(), 0x1600);  // 0x16 is 22
    }
  }

  public static class FirmwareSparkMax extends Firmware {
    public FirmwareSparkMax(SubsystemBase subsystem, CANSparkMax device) {
      super(subsystem, device.getDeviceId(), device.getFirmwareVersion(), 0x01050002);
    }
  }

  private static void banner() {    
    System.out.println("============================TESTING============================");
  }

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
}
