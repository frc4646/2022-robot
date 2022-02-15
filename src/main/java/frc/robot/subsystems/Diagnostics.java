package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleFaults;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.controls.OperatorControls;
import frc.robot.util.DiagnosticState;
import frc.robot.util.Test;

public class Diagnostics extends SmartSubsystem {
  public static Animation toColor(int red, int green, int blue) { return toColor(red, green, blue, .25); }  // TODO tune
  public static Animation toColor(int red, int green, int blue, double percentFade) { return new SingleFadeAnimation(red, green, blue, 0, percentFade, Constants.Diagnostic.LED_COUNT); }
  public static double BRIGHT = 1.0;
  public static double FAST = 1.0;

  public static final Animation 
    OFF = toColor(0, 0, 0),
    BLUE_FADE = toColor(0, 0, 255, .5),  // TODO tune
    BLUE_SOLID = toColor(0, 0, 255),
    RED_FADE = toColor(255, 0, 0, .5),  // TODO tune
    RED_SOLID = toColor(255, 0, 0);

  private final CANdle candle;
  // private final OperatorControls operator;
  private Animation modeDefault = OFF, robotState = OFF;
  private boolean isCriticalIssuePresent = false;
  
  public Diagnostics() {
    candle = new CANdle(Constants.CAN.CANDLE);
    candle.configAllSettings(Constants.Diagnostic.LED_CONFIG, Constants.CAN_TIMEOUT);
    // TODO configure status frames    
    // operator = RobotContainer.CONTROLS.operator;
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void updateDashboard() {
    // TODO reduce calls to CAN HAL?
    // candle.animate(robotState);
    //candle.modulateVBatOutput(joystick.getRightY());
    double rumble = (DriverStation.isDisabled() && isCriticalIssuePresent) ? Constants.Diagnostic.RUMBLE_PERCENT : 0.0;
    // operator.setRumble(true, rumble);  // tune which side is better
    // operator.setRumble(false, rumble);
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    if (isAutonomous) {
      modeDefault = DriverStation.getAlliance() == Alliance.Red ? RED_SOLID : BLUE_SOLID;
    } else {
      modeDefault = OFF;  // Keep drive team sensitive to diagnostics
    }
  }

  @Override
  public void onDisable() {
    modeDefault = DriverStation.getAlliance() == Alliance.Red ? RED_FADE : BLUE_FADE;
  }

  // public void set(int red, int green, int blue) {
  //   robotState = toColor(red, green, blue);  // setLEDs() would also work
  // }

  public void setState(DiagnosticState animation) {
    robotState = animation.diagnostic;
    isCriticalIssuePresent = animation.critical;
  }

  public void setStateOkay() {
    robotState = modeDefault;
    isCriticalIssuePresent = false;
  }

  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 0);
  }

  @Override
  public void runTests() {
    CANdleFaults faults = new CANdleFaults();
    candle.getFaults(faults);

    Test.add("Candle: Fault", !faults.hasAnyFault());

    Animation COLOR_FLOW = new ColorFlowAnimation(128, 20, 70, 0, FAST, Constants.Diagnostic.LED_COUNT, Direction.Forward);
    Animation FIRE = new FireAnimation(BRIGHT, 0.7, Constants.Diagnostic.LED_COUNT, 0.7, 0.5);
    Animation LARSON = new LarsonAnimation(255, 0, 0, 0, FAST, Constants.Diagnostic.LED_COUNT, BounceMode.Front, 7);
    Animation RAINBOW = new RainbowAnimation(BRIGHT, 0.1, Constants.Diagnostic.LED_COUNT);
    Animation RGB_FADE = new RgbFadeAnimation(BRIGHT, FAST, Constants.Diagnostic.LED_COUNT);
    Animation SINGLE_FADE = new SingleFadeAnimation(50, 2, 200, 0, FAST, Constants.Diagnostic.LED_COUNT);
    Animation STROBE = new StrobeAnimation(255, 0, 0, 0, 98.0 / 256.0, Constants.Diagnostic.LED_COUNT);
    Animation TWINKLE = new TwinkleAnimation(255, 0, 0, 0, FAST, Constants.Diagnostic.LED_COUNT, TwinklePercent.Percent6);
    Animation TWINKLE_OFF = new TwinkleOffAnimation(255, 0, 0, 0, FAST, Constants.Diagnostic.LED_COUNT, TwinkleOffPercent.Percent100);
    List<Animation> ALL_ANIMATIONS = Arrays.asList(COLOR_FLOW, FIRE, LARSON, RAINBOW, RGB_FADE, SINGLE_FADE, STROBE, TWINKLE, TWINKLE_OFF);
  
    for(Animation animation : ALL_ANIMATIONS) {
      Test.add("Candle:  " + animation.getClass().getSimpleName(), true);
      candle.animate(animation);
      Timer.delay(5.0);
    }
  }
}
