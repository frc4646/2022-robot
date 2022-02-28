package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.util.Test;

public class ColorSensor extends SmartSubsystem {
  public static enum STATE {
    NOT_PRESENT, CORRECT, WRONG, UNKNOWN_COLOR
  }
  public static class DataCache {
    public STATE state;

    public Color colorRaw;
    public int infraredRaw;  // TODO remove?
    public int distance;
    public ColorMatchResult match;
  }
  
  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatcher;
  private DataCache cache = new DataCache();

  private Color colorAlliance = Constants.ColorSensor.MATCH_RED;
  private Color colorOpponent = Constants.ColorSensor.MATCH_RED;

  public ColorSensor() {
    colorSensor = new ColorSensorV3(Constants.ColorSensor.I2C_PORT);
    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(Constants.ColorSensor.MATCH_BLUE);
    colorMatcher.addColorMatch(Constants.ColorSensor.MATCH_RED);
  }

  @Override
  public void cacheSensors() {
    try {
      if(colorSensor.isConnected()) {
        cache.colorRaw = colorSensor.getColor();
        cache.infraredRaw = colorSensor.getIR();
        cache.distance = colorSensor.getProximity();
        cache.match = colorMatcher.matchClosestColor(cache.colorRaw);
        // TODO switch to matchColor? Incorperates confidence level. Set confidence threshold?  
      }
    } catch (NullPointerException npe) {
      cache.distance = 0;  // TODO handle matcher returns null?
    }

    if(cache.distance < Constants.ColorSensor.DISTANCE_MIN)
      cache.state = STATE.NOT_PRESENT;
    else if(cache.match.color == colorAlliance)
      cache.state = STATE.CORRECT;
    else if(cache.match.color == colorOpponent)
      cache.state = STATE.WRONG;
    else
      cache.state = STATE.UNKNOWN_COLOR;
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putString("Color: State", getState().toString());
    SmartDashboard.putNumber("Color: Distance", cache.distance);
    if (Constants.ColorSensor.TUNING) {
      SmartDashboard.putNumber("Color: Confidence", cache.match.confidence);

      SmartDashboard.putNumber("Color: Red", cache.colorRaw.red);
      SmartDashboard.putNumber("Color: Green", cache.colorRaw.green);
      SmartDashboard.putNumber("Color: Blue", cache.colorRaw.blue);
      SmartDashboard.putNumber("Color: Infrared", cache.infraredRaw);
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
    colorAlliance = alliance == Alliance.Red ? Constants.ColorSensor.MATCH_RED : Constants.ColorSensor.MATCH_BLUE;
    colorOpponent = alliance == Alliance.Red ? Constants.ColorSensor.MATCH_BLUE : Constants.ColorSensor.MATCH_RED;
  }

  @Override
  public void runTests() {
    Test.add(this, "Is Connected", colorSensor.isConnected());
  }
}
