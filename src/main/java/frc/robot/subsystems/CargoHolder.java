package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorSensor.STATE;

/***
 * In it's current state, it will just monitor the state of the sensors for where cargo is at
 * Initial credits to team 95: https://github.com/first95/FRC2022/blob/main/Lamarr/src/main/java/frc/robot/commands/ControlCargoHandling.java
 */
public class CargoHolder extends SmartSubsystem {
  private enum State {
    IDLE, INTAKING, SHOOTING, EJECT_SHOOT, EJECT_INTAKE
  }

  private State currentState = State.IDLE;

  private boolean wasShooterLoaded = false;
  private int ejectionTimer = 0;

  @Override
  public void cacheSensors() {
    STATE state = RobotContainer.COLOR_SENSOR.getState();
    boolean isShooterLoaded = RobotContainer.FEEDER.isShooterLoaded();
    boolean shootRequested = RobotContainer.SHOOTER.isShooting(); // not technically correct, but works for monitoring
  
    switch (currentState) {
      case IDLE:
        // here if we have our cargo loaded for the shooter and may or may not have our cargo in the intake
        if ((state == STATE.CORRECT) && !isShooterLoaded) {
          currentState = State.INTAKING;  // got a lower cargo and need to intake it to the top
        }
        if ((state == STATE.WRONG) && !isShooterLoaded) {
          currentState = State.EJECT_SHOOT;  // got the wrong cargo and need to shoot it out the top
        }
        if ((state == STATE.WRONG) && isShooterLoaded) {
          currentState = State.EJECT_INTAKE;  // got the wrong cargo and have one already, so shoot it out the intake
        }
        if (shootRequested) {
          currentState = State.SHOOTING;  // need to shoot!
        }
        break;
      case INTAKING:
        // here if we dont have cargo loaded for the shooter and are trying to intake
        if (isShooterLoaded) {
          currentState = State.IDLE;  // we now have a cargo ready for the shooter, so idle
        }
        if ((state == STATE.WRONG) && !isShooterLoaded) {
          currentState = State.EJECT_SHOOT;   // got the wrong cargo and need to shoot it out the top
        }        
        if ((state == STATE.WRONG) && isShooterLoaded) {
          currentState = State.EJECT_INTAKE;  // got the wrong cargo and have one already, so shoot it out the intake
        }
        if (shootRequested) {
          currentState = State.SHOOTING;  // need to shoot!
        }
        break;
      case SHOOTING:
        // a shot was requeested, so empty our guts
        if (!shootRequested) {
          currentState = State.IDLE;  // no longer need to shoot, so go chill
        }
        break;
      case EJECT_SHOOT:
        // we need to eject the top cargo, so do a single shot
        if (!isShooterLoaded && wasShooterLoaded) {
          currentState = State.IDLE;  // we had a cargo and shot it, so be done with it!
        }
        break;
      case EJECT_INTAKE:
        if ((state == STATE.CORRECT) || (state == STATE.NOT_PRESENT) || (ejectionTimer > 0)) {
          ejectionTimer++;  // only wait for X time if it's not the wrong cargo
        }
        if (ejectionTimer >= 10) {
          ejectionTimer = 0;
          currentState = State.IDLE;  // and just wait for 10 loops (?)
        }
        break;
    }
    wasShooterLoaded = isShooterLoaded;
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putString("CargoHolder: State", currentState.toString());
  }
}
