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

  public static enum CargoColor {
    NONE, CORRECT, WRONG
  }
  public static class DataCacheDetails {
    public Color detectedColor;
    public ColorMatchResult match;
    public boolean hasCargo;

    public int ir;
    public int proximity;
  }

  public static class DataCache {
    public CargoColor color;

    public DataCacheDetails details = new DataCacheDetails();
  }
  
  private DataCache cache = new DataCache();
  
  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatcher;

  private Alliance alliance, lastAlliance;
  private Color allianceColor;
  private Color opponentColor;


  public ColorSensor() {
    colorSensor = new ColorSensorV3(Constants.ColorSensor.I2C_PORT);

    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(Constants.ColorSensor.BLUE_CARGO_TARGET);
    colorMatcher.addColorMatch(Constants.ColorSensor.RED_CARGO_TARGET); 
    
    lastAlliance = Alliance.Invalid;
  }


  @Override
  public void cacheSensors() {
    alliance = DriverStation.getAlliance();
    if(lastAlliance != alliance)
    {
      // save alliance color target
      if(alliance == Alliance.Red)
      {
        allianceColor = Constants.ColorSensor.RED_CARGO_TARGET;
        opponentColor = Constants.ColorSensor.BLUE_CARGO_TARGET;
      }
      else
      {
        allianceColor = Constants.ColorSensor.BLUE_CARGO_TARGET;
        opponentColor = Constants.ColorSensor.RED_CARGO_TARGET;
      }
      lastAlliance = alliance;
    }

    // get updated data from the sensor
    try {
      if(colorSensor.isConnected())
      {
        cache.details.detectedColor = colorSensor.getColor();
        cache.details.ir = colorSensor.getIR();
        cache.details.proximity = colorSensor.getProximity();
        cache.details.match = colorMatcher.matchClosestColor(cache.details.detectedColor);
      }
    } catch (NullPointerException npe) {
      cache.details.proximity = 0;
    }
    
    // if the detected distance is close enough, we have the cargo
    // value is larger when closer, smaller when far away
    cache.details.hasCargo = cache.details.proximity >= Constants.ColorSensor.PROXIMITY_TO_CARGO;

    // set the color state
    if(!cache.details.hasCargo)
      cache.color = CargoColor.NONE;
    else if(cache.details.match.color == allianceColor)
      cache.color = CargoColor.CORRECT;
    else if(cache.details.match.color == opponentColor)
      cache.color = CargoColor.WRONG;
    else
      cache.color = CargoColor.NONE;
  }

  @Override
  public void updateDashboard() {
    
    SmartDashboard.putBoolean("ColorSensor/Detected", isCargoDetected());
    SmartDashboard.putBoolean("ColorSensor/Correct", getCargoColor() == CargoColor.CORRECT);
    SmartDashboard.putBoolean("ColorSensor/Wrong", getCargoColor() == CargoColor.WRONG);

    if (Constants.ColorSensor.TUNING) {
      String colorString;
      if (cache.details.match.color == Constants.ColorSensor.BLUE_CARGO_TARGET) {
        colorString = "Blue";
      } else if (cache.details.match.color == Constants.ColorSensor.RED_CARGO_TARGET) {
        colorString = "Red";
      } else {
        colorString = "Unknown";
      }
      SmartDashboard.putBoolean("ColorSensor/HasCargo", cache.details.hasCargo);
      SmartDashboard.putString("ColorSensor/Color", colorString);
      SmartDashboard.putNumber("ColorSensor/Confidence", cache.details.match.confidence);

      SmartDashboard.putNumber("ColorSensor/Red", cache.details.detectedColor.red);
      SmartDashboard.putNumber("ColorSensor/Green", cache.details.detectedColor.green);
      SmartDashboard.putNumber("ColorSensor/Blue", cache.details.detectedColor.blue);
      SmartDashboard.putNumber("ColorSensor/IR", cache.details.ir);
      SmartDashboard.putNumber("ColorSensor/Proximity", cache.details.proximity);
    }

  }

  public boolean isCargoDetected()
  {
    return cache.color != CargoColor.NONE;
  }

  public CargoColor getCargoColor()
  {
    return cache.color;
  }


  @Override
  public void runTests() {
    boolean isConnected = colorSensor.isConnected();

    Test.add(this, "Color Sensor - Is Connected", isConnected);
  }
}
