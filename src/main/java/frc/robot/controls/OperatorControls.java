package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.hood.HoodExtend;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.LoadCargo;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.sequence.StopShoot;
import frc.robot.commands.sequence.StowIntake;
import frc.team254.CardinalDirection;
import frc.team254.util.DelayedBoolean;

public class OperatorControls {
  private final double DPAD_DELAY = 0.02;

  private final XboxController operator;
  private final Trigger padUp, padDown, padLeft, padRight, leftStickL, leftStickR, leftStickOff;
  private final JoystickButton buttonA, buttonB, buttonX, buttonY, bumperL, bumperR, Fn, start;
  private DelayedBoolean validDPAD;
  private CardinalDirection lastCardinal = CardinalDirection.NONE;
  
  public OperatorControls() {
    operator = new XboxController(2);
    buttonA = makeButton(Button.kA); buttonB = makeButton(Button.kB); buttonX = makeButton(Button.kX); buttonY = makeButton(Button.kY);
    bumperL = makeButton(Button.kLeftBumper); bumperR = makeButton(Button.kRightBumper);
    Fn = makeButton(Button.kBack); start = makeButton(Button.kStart);
    padUp = getUpDPAD(); padDown = getDownDPAD(); padLeft = getLeftDPAD(); padRight = getRightDPAD();
    leftStickL = getLeftStickLeft(); leftStickR = getLeftStickRight(); leftStickOff = getLeftStickOff();
    validDPAD = new DelayedBoolean(Timer.getFPGATimestamp(), DPAD_DELAY);

    // Agitator
    buttonA.whenPressed(new AgitateOpenLoop(1.0));
    buttonA.whenReleased(new AgitateOpenLoop(0.0));
    buttonA.and(Fn).whenActive(new AgitateOpenLoop(-1.0));

    // Climber
    // start.and(Fn) TODO climb mode

    // Intake
    buttonY.whenPressed(new ParallelCommandGroup(new DeployIntake(), new LoadCargo()));
    buttonY.whenReleased(new ParallelCommandGroup(new StowIntake(), new FeederOpenLoop(0.0)));
    buttonY.and(Fn).whenActive(new IntakeActivate(-Constants.Intake.OPEN_LOOP));
    buttonX.debounce(.1).whenActive(new ParallelCommandGroup(new DeployIntake(), new LoadCargo()));  // TODO tune if debounce better
    buttonX.debounce(.1).whenInactive(new ParallelCommandGroup(new StowIntake(), new FeederOpenLoop(0.0)));
    buttonX.and(Fn).whenActive(new IntakeActivate(-Constants.Intake.OPEN_LOOP));
    buttonB.whenPressed(new HoodExtend(true));
    buttonB.whenReleased(new HoodExtend(false));

    // Shooter
    bumperL.whenPressed(new ShootOpenLoop());
    bumperL.whenReleased(new StopShoot().alongWith(new IntakeActivate(0.0), new AgitateOpenLoop(0.0)));
    bumperR.whenPressed(new LoadCargo());
    bumperR.whenReleased(new FeederOpenLoop(0.0));

    // Turret
    // leftStickL.whenActive(new TurretOpenLoop(Constants.Turret.OPEN_LOOP));
    // leftStickR.whenActive(new TurretOpenLoop(-Constants.Turret.OPEN_LOOP));
    // leftStickOff.whenActive(new TurretOpenLoop(0.0));
    // TODO dpad turret snap to robot angle
    // TODO move to zero
  }

  public double getTurretStick() {
      return operator.getRawAxis(XboxController.Axis.kLeftX.value);
  }
  
  public CardinalDirection getTurretSnap() {
    int dPad = operator.getPOV();
    CardinalDirection newCardinal = dPad == -1 ? CardinalDirection.NONE : CardinalDirection.findClosest(Rotation2d.fromDegrees(-dPad));
    if (newCardinal != CardinalDirection.NONE && CardinalDirection.isDiagonal(newCardinal)) {
      newCardinal = lastCardinal;
    }
    boolean valid = validDPAD.update(Timer.getFPGATimestamp(), newCardinal != CardinalDirection.NONE && (lastCardinal == CardinalDirection.NONE || newCardinal == lastCardinal));
    if (valid) {
      if (lastCardinal == CardinalDirection.NONE) {
        lastCardinal = newCardinal;
      }
      return lastCardinal;
    } else {
      lastCardinal = newCardinal;
    }
    return CardinalDirection.NONE;
  }

  public void setRumble(boolean wantLeft, double percent) {
    RumbleType half = wantLeft ? RumbleType.kLeftRumble : RumbleType.kRightRumble;
    operator.setRumble(half, percent);
  }

  protected Trigger getLeftStickLeft() {
    return new Trigger() {
      @Override
      public boolean get() { return getTurretStick() <= -Constants.Turret.OPEN_LOOP_DEADBAND; }
    };
  }
  
  protected Trigger getLeftStickRight() {
    return new Trigger() {
      @Override
      public boolean get() { return getTurretStick() >= Constants.Turret.OPEN_LOOP_DEADBAND; }
    };
  }

  protected Trigger getLeftStickOff() {
    return new Trigger() {
      @Override
      public boolean get() { return Math.abs(getTurretStick()) < Constants.Turret.OPEN_LOOP_DEADBAND; }
    };
  }

  protected Trigger getUpDPAD() {
    return new Trigger() {
      @Override
      public boolean get() { int position = operator.getPOV(0); return position == 315 || position == 0 || position == 45; }
    };
  }

  protected Trigger getDownDPAD() {
    return new Trigger() {
      @Override
      public boolean get() { int position = operator.getPOV(0); return position == 135 || position == 180 || position == 225; }
    };
  }

  protected Trigger getLeftDPAD() {
    return new Trigger() {
      @Override
      public boolean get() { int position = operator.getPOV(0); return position == 270; }
    };
  }

  protected Trigger getRightDPAD() {
    return new Trigger() {
      @Override
      public boolean get() { int position = operator.getPOV(0); return position == 90; }
    };
  }  

  protected JoystickButton makeButton(Button button) {
    return new JoystickButton(operator, button.value);
  }
}
