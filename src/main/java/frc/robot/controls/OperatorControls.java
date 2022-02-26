package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.climber.ClimberArms;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.LoadCargo;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.sequence.StopShoot;
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

    // Climber
    buttonB.whenPressed(new ClimberArms(true));
    buttonB.whenReleased(new ClimberArms(false));
    // buttonB.and(Fn).whenPressed(new ClimberRelease());

	// Hood
    // aimLob.whenActive(new HoodExtend(false));  // TODO test these
    // aimFar.whenActive(new HoodExtend(true));

    // Intake
    buttonY.whenPressed(new ParallelCommandGroup(new DeployIntake(), new LoadCargo()));
    buttonY.whenReleased(new ParallelCommandGroup(new StowIntake(), new FeederOpenLoop(0.0)));
    buttonY.and(Fn).whenActive(new IntakeActivate(-Constants.Intake.OPEN_LOOP));
    // TODO reverse intake should also reverse agitators??
    buttonX.debounce(.1).whenActive(new ParallelCommandGroup(new DeployIntake(), new LoadCargo()));  // TODO tune if debounce better
    buttonX.debounce(.1).whenInactive(new ParallelCommandGroup(new StowIntake(), new FeederOpenLoop(0.0)));
    buttonX.and(Fn).whenActive(new IntakeActivate(-Constants.Intake.OPEN_LOOP));
    // TODO reverse intake should also reverse agitators??

    // Shooter
    bumperL.whenPressed(new ShootOpenLoop());
    bumperL.whenReleased(new StopShoot().alongWith(new IntakeActivate(0.0), new AgitateOpenLoop(0.0)));
    bumperR.whenPressed(new LoadCargo());
    bumperR.whenReleased(new FeederOpenLoop(0.0));

    // Turret
    // TODO move to zero
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
