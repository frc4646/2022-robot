package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.team4646.StabilityCounter;
import frc.team4646.Test;

public class ColorSensor extends SmartSubsystem {
  public enum STATE {
    NOT_PRESENT, CORRECT, WRONG, UNKNOWN_COLOR
  }
  private class DataCache {
    public STATE state;

    public Color colorRaw;
    public int distance;
    public ColorMatchResult match;
  }
  
  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatcher;
  private final StabilityCounter stabilityCorrect = new StabilityCounter(1);
  private final StabilityCounter stabilityWrong = new StabilityCounter(1);
  private final DataCache cache = new DataCache();

  private Color colorAlliance = Constants.COLORSENSOR.MATCH_RED;
  private Color colorOpponent = Constants.COLORSENSOR.MATCH_RED;
  public BooleanSupplier isWrongCargo = () -> isWrongCargo();

  public ColorSensor() {
    colorSensor = new ColorSensorV3(Constants.COLORSENSOR.I2C_PORT);
    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(Constants.COLORSENSOR.MATCH_BLUE);
    colorMatcher.addColorMatch(Constants.COLORSENSOR.MATCH_RED);
  }

  @Override
  public void cacheSensors() {
    try {
      if(colorSensor.isConnected()) {
        cache.colorRaw = colorSensor.getColor();
        cache.distance = colorSensor.getProximity();
        cache.match = colorMatcher.matchClosestColor(cache.colorRaw);
        // TODO switch to matchColor? Incorperates confidence level. Set confidence threshold?  
      }
      else{
        cache.state = STATE.NOT_PRESENT;
      }
    } catch (NullPointerException npe) {
      cache.distance = 0;  // TODO handle matcher returns null?
    }

    if(cache.distance < Constants.COLORSENSOR.DISTANCE_MIN)
      cache.state = STATE.NOT_PRESENT;
    else if(cache.match.color == colorAlliance)
      cache.state = STATE.CORRECT;
    else if(cache.match.color == colorOpponent)
      cache.state = STATE.WRONG;
    else
      cache.state = STATE.UNKNOWN_COLOR;
    
    stabilityCorrect.calculate(cache.state == STATE.CORRECT);
    stabilityWrong.calculate(cache.state == STATE.WRONG);
  }

  @Override
  public void updateDashboard(boolean showDetails) {
    if (showDetails) {
      SmartDashboard.putBoolean("Color: Correct", getState() == STATE.CORRECT);
      SmartDashboard.putString("Color: State", getState().toString());
      SmartDashboard.putNumber("Color: Distance", cache.distance);
      SmartDashboard.putNumber("Color: Counts Correct", stabilityCorrect.counts());
      SmartDashboard.putNumber("Color: Counts Wrong", stabilityWrong.counts());
    }
    if (Constants.TUNING.COLORSENSOR) {
      SmartDashboard.putNumber("Color: Confidence", cache.match.confidence);
      SmartDashboard.putNumber("Color: Red", cache.colorRaw.red);
      SmartDashboard.putNumber("Color: Green", cache.colorRaw.green);
      SmartDashboard.putNumber("Color: Blue", cache.colorRaw.blue);
    }
  }

  @Override
  public void onEnable(boolean isAutonomous) {
    updateAlliance();
  }

  @Override
  public void onDisable() {
    updateAlliance();
  }

  public STATE getState() { return cache.state; }
  public boolean isCargoPresent() { return cache.state != STATE.NOT_PRESENT; }
  public boolean isCargoAbsent() { return cache.state == STATE.NOT_PRESENT; }
  public boolean isCorrectCargo() { return cache.state == STATE.CORRECT; }
  public boolean isWrongCargo() { return cache.state == STATE.WRONG; }

  private void updateAlliance() {
    Alliance alliance = DriverStation.getAlliance();
    colorAlliance = alliance == Alliance.Red ? Constants.COLORSENSOR.MATCH_RED : Constants.COLORSENSOR.MATCH_BLUE;
    colorOpponent = alliance == Alliance.Red ? Constants.COLORSENSOR.MATCH_BLUE : Constants.COLORSENSOR.MATCH_RED;
  }

  @Override
  public void runTests() {
    Test.add(this, "Is Connected", colorSensor.isConnected());
  }
}
