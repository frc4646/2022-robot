package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorSensor.CargoColor;


/***
 * In it's current state, it will just monitor the state of the sensors for where cargo is at
 * Initial credits to team 95: https://github.com/first95/FRC2022/blob/main/Lamarr/src/main/java/frc/robot/commands/ControlCargoHandling.java
 */
public class CargoHolder extends SmartSubsystem {

  private enum State {
    IDLE, INTAKING, SHOOTING, EJECT_SHOOT, EJECT_INTAKE
  }

  // FSM variables
  private State currentState;

  private boolean wasShooterLoaded;
  private int ejectionTimer;

  public CargoHolder()
  {
    currentState = State.IDLE;
    ejectionTimer = 0;
    wasShooterLoaded = false;
  }

  @Override
  public void cacheSensors() // or periodic?
  {
    CargoColor color = RobotContainer.COLOR_SENSOR.getCargoColor();
    boolean isShooterLoaded = RobotContainer.FEEDER.isCargoPresent();
    boolean shootRequested = RobotContainer.SHOOTER.isShooting(); // not technically correct, but works for monitoring
    
    switch (currentState) {
      case IDLE:
        // here if we have our cargo loaded for the shooter
        // and may or may not have our cargo in the intake

        // got a lower cargo and need to intake it to the top
        if ((color == CargoColor.CORRECT) && !isShooterLoaded)
        {
          currentState = State.INTAKING;
        }
        // got the wrong cargo and need to shoot it out the top
        if ((color == CargoColor.WRONG) && !isShooterLoaded)
        {
          currentState = State.EJECT_SHOOT;
        }
        // got the wrong cargo and have one already, so shoot it out the intake
        if ((color == CargoColor.WRONG) && isShooterLoaded)
        {
          currentState = State.EJECT_INTAKE;
        }

        // need to shoot!
        if (shootRequested)
        {
          currentState = State.SHOOTING;
        }
        break;

      case INTAKING:
        // here if we dont have cargo loaded for the shooter
        // and are trying to intake

        // we now have a cargo ready for the shooter, so idle
        if (isShooterLoaded) {
          currentState = State.IDLE;
        }
        
        // got the wrong cargo and need to shoot it out the top
        if ((color == CargoColor.WRONG) && !isShooterLoaded) {
          currentState = State.EJECT_SHOOT;
        }
        
        // got the wrong cargo and have one already, so shoot it out the intake
        if ((color == CargoColor.WRONG) && isShooterLoaded) {
          currentState = State.EJECT_INTAKE;
        }

        // need to shoot!
        if (shootRequested) {
          currentState = State.SHOOTING;
        }
        break;

      case SHOOTING:
        // a shot was requeested, so empty our guts

        // no longer need to shoot, so go chill
        if (!shootRequested) {
          currentState = State.IDLE;
        }
        break;

      case EJECT_SHOOT:
        // we need to eject the top cargo, so do a single shot

        // we had a cargo and shot it, so be done with it!
        if (!isShooterLoaded && wasShooterLoaded) {
          currentState = State.IDLE;
        }
        break;

      case EJECT_INTAKE:
        // only wait for X time if it's not the wrong cargo
        if ((color == CargoColor.CORRECT) || (color == CargoColor.NONE) || (ejectionTimer > 0)) {
          ejectionTimer++;
        }

        // and just wait for 10 loops (?)
        if (ejectionTimer >= 10) {
          ejectionTimer = 0;
          currentState = State.IDLE;
        }
        break;
    }

    wasShooterLoaded = isShooterLoaded;
  }

  @Override
  public void updateDashboard()
  {
    switch (currentState) {
      case IDLE:
        SmartDashboard.putString("CargoHolder/State", "IDLE");
        break;
      case INTAKING:
        SmartDashboard.putString("CargoHolder/State", "INTAKING");
        break;
      case SHOOTING:
        SmartDashboard.putString("CargoHolder/State", "SHOOTING");
        break;
      case EJECT_SHOOT:
        SmartDashboard.putString("CargoHolder/State", "EJECT_SHOOT");
        break;
      case EJECT_INTAKE:
        SmartDashboard.putString("CargoHolder/State", "EJECT_INTAKE");
        break;
    }
  }
}
