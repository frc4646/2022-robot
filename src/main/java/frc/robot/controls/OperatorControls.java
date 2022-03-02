package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.climber.ClimberArms;
import frc.robot.commands.feeder.FeederPosition;
import frc.robot.commands.sequence.ClimbMode;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.ExhaustIntake;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.sequence.ShootVision;
import frc.robot.commands.sequence.StowIntake;

public class OperatorControls {
  private final int TRIGGER_L = 2, TRIGGER_R = 3;
  private final double TRIGGER_DEADBAND = 0.2;
  private final XboxController operator;
  private final JoystickButton buttonA, buttonB, buttonX, buttonY, bumperL, bumperR, Fn, start;
  private final Trigger aimLob, aimFar;

  public OperatorControls() {
    operator = new XboxController(2);
    buttonA = makeButton(Button.kA); buttonB = makeButton(Button.kB); buttonX = makeButton(Button.kX); buttonY = makeButton(Button.kY);
    bumperL = makeButton(Button.kLeftBumper); bumperR = makeButton(Button.kRightBumper);
    Fn = makeButton(Button.kBack); start = makeButton(Button.kStart);
    aimLob = new Trigger() { @Override public boolean get() { return getAimLob(); } };
    aimFar = new Trigger() { @Override public boolean get() { return getAimFar(); } };
  }

  public void configureButtons() {
    // Climber
    start.whenPressed(new ClimberArms(true));
    start.whenReleased(new ClimberArms(false));
    buttonX.toggleWhenPressed(new ClimbMode());
    // Fn.whenPressed(new ClimberZero() or new ClimberOverrideLimits());  // auto zero will do wrong thing if spool rolls over

    // Hood
    // aimLob.whenActive(new HoodExtend(false));  // TODO test these
    // aimFar.whenActive(new HoodExtend(true));

    // Intake
    buttonY.whenPressed(new DeployIntake());
    buttonY.whenReleased(new StowIntake());
    buttonY.and(Fn).whenActive(new ExhaustIntake());  // TODO just on regular button?
    buttonY.and(Fn).whenInactive(new StowIntake());
    buttonA.whenActive(new ExhaustIntake());
    buttonA.whenInactive(new StowIntake());
    
    // Shooter
    bumperL.whenPressed(new ShootOpenLoop());
    bumperR.whenPressed(new ShootVision());

    // Testing
    buttonB.whenActive(new FeederPosition(1.0));
    buttonB.whenInactive(new FeederPosition(-1.0));
  }

  public boolean getAimLob() { return operator.getRawAxis(TRIGGER_L) > TRIGGER_DEADBAND; }
  public boolean getAimFar() { return operator.getRawAxis(TRIGGER_R) > TRIGGER_DEADBAND; }
  public double getClimberStick() { return -operator.getRawAxis(XboxController.Axis.kRightY.value); }
  public double getTurretStick() { return -operator.getRawAxis(XboxController.Axis.kLeftX.value); }
  public int getTurretSnap() { return operator.getPOV(); }
  public boolean getFn() { return operator.getBackButton(); }

  public void setRumble(boolean wantLeft, double percent) {
    RumbleType half = wantLeft ? RumbleType.kLeftRumble : RumbleType.kRightRumble;
    operator.setRumble(half, percent);
  }

  protected JoystickButton makeButton(Button button) {
    return new JoystickButton(operator, button.value);
  }
}
