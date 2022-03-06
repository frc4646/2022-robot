package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.subsystems.Canifier.COLOR;
import frc.robot.util.DiagnosticState;
import frc.robot.util.Test;

public class Diagnostics extends SmartSubsystem {


  public static Animation toColor(int red, int green, int blue, double percentFade) { return new SingleFadeAnimation(red, green, blue, 0, percentFade, Constants.DIAGNOSTICS.LED_COUNT); }
  public static double BRIGHT = 1.0, DIM = 0.2;
  public static double FAST = 1.0, SLOW = 0.2, STATIC = 0.0;

  public static final Animation 
    OFF = toColor(0, 0, 0, 0.0),
    BLUE_FADE = toColor(0, 0, 255, 1.0),  // TODO tune
    BLUE_SOLID = toColor(0, 0, 255, 0.5),
    RED_FADE = toColor(255, 0, 0, 1.0),  // TODO tune
    RED_SOLID = toColor(255, 0, 0, 0.5);

  public static final Canifier.COLOR 
    COLOR_OFF = new Canifier.COLOR(0, 0, 0);

  private final CANdle candle;
  private final Canifier canifier;
  private final OperatorControls operator = RobotContainer.CONTROLS.getOperator();
  private Animation modeDefault = OFF, robotState = OFF;
  private boolean isCriticalIssuePresent = false;
  private COLOR shooterColor = COLOR_OFF;
  
  public Diagnostics() {
    candle = new CANdle(Constants.CAN.CANDLE);
    candle.configAllSettings(Constants.DIAGNOSTICS.LED_CONFIG, Constants.CAN_TIMEOUT);

    canifier = RobotContainer.CANIFIER;

    // TODO configure status frames
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void updateDashboard() {
    // TODO reduce calls to CAN HAL?
    candle.animate(robotState);
    canifier.setLEDs(shooterColor);
    double rumble = (DriverStation.isDisabled() && isCriticalIssuePresent) ? Constants.DIAGNOSTICS.RUMBLE_PERCENT : 0.0;
    operator.setRumble(true, rumble);  // tune which side is better
    operator.setRumble(false, rumble);
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    if (isAutonomous) {
      modeDefault = DriverStation.getAlliance() == Alliance.Red ? RED_FADE : BLUE_FADE;
    } else {
      modeDefault = OFF;  // Keep drive team sensitive to diagnostics
    }
  }

  @Override
  public void onDisable() {
    modeDefault = DriverStation.getAlliance() == Alliance.Red ? RED_SOLID : BLUE_SOLID;
  }

  // public void set(int red, int green, int blue) {
  //   robotState = toColor(red, green, blue);  // setLEDs() would also work
  // }

  public void setState(DiagnosticState animation, Canifier.COLOR color) {
    robotState = animation.diagnostic;
    isCriticalIssuePresent = animation.critical;
    shooterColor = color;
  }

  public void setStateOkay() {
    robotState = modeDefault;
    isCriticalIssuePresent = false;
    shooterColor = Constants.DIAGNOSTICS.COLOR_OFF;
  }

  @Override
  public void runTests() {
    CANdleFaults faults = new CANdleFaults();
    candle.getFaults(faults);

    Test.add(this, "Candle - Fault", !faults.hasAnyFault());

    System.out.println(String.format("Frame 1: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General)));
    System.out.println(String.format("Frame 2: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_2_Startup)));
    System.out.println(String.format("Frame 3: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_3_FirmwareApiStatus)));
    System.out.println(String.format("Frame 4: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_4_ControlTelem)));
    System.out.println(String.format("Frame 5: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_5_PixelPulseTrain)));
    System.out.println(String.format("Frame 6: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_6_BottomPixels)));
    System.out.println(String.format("Frame 7: %d", candle.getStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_7_TopPixels)));

    Animation COLOR_FLOW = new ColorFlowAnimation(255, 0, 0, 0, SLOW, Constants.DIAGNOSTICS.LED_COUNT, Direction.Forward);
    Animation FIRE = new FireAnimation(BRIGHT, SLOW, Constants.DIAGNOSTICS.LED_COUNT, 0.8, 0.2);
    Animation LARSON = new LarsonAnimation(255, 0, 0, 0, SLOW, Constants.DIAGNOSTICS.LED_COUNT, BounceMode.Front, 4);
    Animation RAINBOW = new RainbowAnimation(BRIGHT, SLOW, Constants.DIAGNOSTICS.LED_COUNT);
    Animation RGB_FADE = new RgbFadeAnimation(BRIGHT, FAST, Constants.DIAGNOSTICS.LED_COUNT);
    Animation SINGLE_FADE = new SingleFadeAnimation(255, 0, 0, 0, STATIC, Constants.DIAGNOSTICS.LED_COUNT);
    Animation STROBE = new StrobeAnimation(255, 0, 0, 0, STATIC, Constants.DIAGNOSTICS.LED_COUNT);
    Animation TWINKLE = new TwinkleAnimation(255, 0, 0, 0, STATIC, Constants.DIAGNOSTICS.LED_COUNT, TwinklePercent.Percent100);
    Animation TWINKLE_OFF = new TwinkleOffAnimation(255, 0, 0, 0, STATIC, Constants.DIAGNOSTICS.LED_COUNT, TwinkleOffPercent.Percent100);
    List<Animation> ALL_ANIMATIONS = Arrays.asList(COLOR_FLOW, FIRE, LARSON, RAINBOW, RGB_FADE, SINGLE_FADE, STROBE, TWINKLE, TWINKLE_OFF);
  
    // for(Animation animation : ALL_ANIMATIONS) {
    //   Test.add(animation.getClass().getSimpleName(), true);
    //   candle.animate(animation);
    //   Timer.delay(5.0);
    // }
  }
}
