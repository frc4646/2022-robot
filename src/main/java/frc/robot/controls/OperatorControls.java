package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.climber.ClimberExtend;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.ClimberEnableLimits;
import frc.robot.commands.sequence.ClimbMode;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.shooter.ShooterOpenLoop;

public class OperatorControls {
  private final int TRIGGER_L = 2, TRIGGER_R = 3;
  private final double TRIGGER_DEADBAND = 0.2;
  private final XboxController operator;
  private final JoystickButton buttonA, buttonB, buttonX, buttonY, bumperL, bumperR, Fn, start;

  // private final Trigger exhaustShoot = new Trigger(RobotContainer.ROBOT_STATE::isAutoExhaustWanted);
  // private final Trigger climbMode;

  public OperatorControls() {
    operator = new XboxController(2);
    buttonA = makeButton(Button.kA); buttonB = makeButton(Button.kB); buttonX = makeButton(Button.kX); buttonY = makeButton(Button.kY);
    bumperL = makeButton(Button.kLeftBumper); bumperR = makeButton(Button.kRightBumper);
    Fn = makeButton(Button.kBack); start = makeButton(Button.kStart);
    // climbMode = new Trigger(RobotContainer.CLIMBER::isInClimbMode);
  }

  public void configureButtons() {
    // Climber
    start.whenPressed(new ClimberExtend(true));  // TODO move to alt buttons when in climb mode?
    start.whenReleased(new ClimberExtend(false));
    Fn.whenPressed(new ClimberEnableLimits(false));
    Fn.whenReleased(new ClimberEnableLimits(true));
    buttonX.toggleWhenPressed(new ClimbMode());  // TODO move to start button?
    // buttonX.whenPressed(new ClimberZero());
    
    // Shooter
    bumperL.whenPressed(new ShootOpenLoop());
    bumperL.whenReleased(new ShooterOpenLoop());
    bumperR.whenPressed(new ShootVision());
    bumperR.whenReleased(new ShooterOpenLoop());
  }

  public boolean getAimLob() { return operator.getRawAxis(TRIGGER_L) > TRIGGER_DEADBAND; }
  public boolean getAimFar() { return operator.getRawAxis(TRIGGER_R) > TRIGGER_DEADBAND; }
  public double getClimberStick() { return -operator.getRawAxis(XboxController.Axis.kRightY.value); }
  public double getShooterTrim() { return -operator.getRawAxis(XboxController.Axis.kLeftY.value); }
  public double getTurretStick() { return operator.getRawAxis(XboxController.Axis.kLeftX.value); }
  public int getTurretSnap() {
    int pov = operator.getPOV();
    if(pov == -1) {
      return -1;
    }
    return  (pov + 180) % 360 - 180; // flip the direction
  }
  public boolean getFn() { return operator.getBackButton(); }

  public void setRumble(boolean wantLeft, double percent) {
    operator.setRumble(wantLeft ? RumbleType.kLeftRumble : RumbleType.kRightRumble, percent);
  }

  protected JoystickButton makeButton(Button button) {
    return new JoystickButton(operator, button.value);
  }
}
