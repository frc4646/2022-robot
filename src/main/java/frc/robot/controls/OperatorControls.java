package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.agitator.AgitateOpenLoop;
import frc.robot.commands.agitator.AgitatorAuto;
import frc.robot.commands.feeder.FeederOpenLoop;
import frc.robot.commands.hood.HoodExtend;
import frc.robot.commands.intake.IntakeActivate;
import frc.robot.commands.sequence.DeployIntake;
import frc.robot.commands.sequence.LoadCargo;
import frc.robot.commands.sequence.ShootOpenLoop;
import frc.robot.commands.sequence.StopShoot;
import frc.robot.commands.sequence.StowIntake;
import frc.robot.commands.turret.TurretOpenLoop;

public class OperatorControls {
  private final double LEFT_DEADBAND = 0.8;
  private final XboxController operator;
  
  public OperatorControls() {
    operator = new XboxController(2);
    JoystickButton inPit = makeButton(Button.kBack);  // TODO use with .and()

    // Agitator
    makeButton(Button.kA).whenPressed(new AgitateOpenLoop(1.0));
    makeButton(Button.kA).whenReleased(new AgitateOpenLoop(0.0));
    getDownDPAD().whenActive(new AgitateOpenLoop(-0.5));
    getDownDPAD().whenInactive(new AgitateOpenLoop(0.0));
    getLeftDPAD().whenActive(new AgitatorAuto(0.5));
    getLeftDPAD().whenInactive(new AgitateOpenLoop(0.0));

    // Intake
    makeButton(Button.kY).whenPressed(new ParallelCommandGroup(new DeployIntake(), new LoadCargo()));
    makeButton(Button.kY).whenReleased(new ParallelCommandGroup(new StowIntake(), new FeederOpenLoop(0.0)));
    makeButton(Button.kX).debounce(.1).whenActive(new ParallelCommandGroup(new DeployIntake(), new LoadCargo()));
    makeButton(Button.kX).debounce(.1).whenInactive(new ParallelCommandGroup(new StowIntake(), new FeederOpenLoop(0.0)));
    makeButton(Button.kB).whenPressed(new HoodExtend(true));
    makeButton(Button.kB).whenReleased(new HoodExtend(false));
    getUpDPAD().whenActive(new IntakeActivate(-Constants.Intake.OPEN_LOOP_PERCENT));
    getUpDPAD().whenInactive(new IntakeActivate(0.0));

    // Shooter
    makeButton(Button.kLeftBumper).whenPressed(new ShootOpenLoop());
    makeButton(Button.kLeftBumper).whenReleased(new StopShoot().alongWith(new IntakeActivate(0.0), new AgitateOpenLoop(0.0)));
    makeButton(Button.kRightBumper).whenPressed(new LoadCargo());
    makeButton(Button.kRightBumper).whenReleased(new FeederOpenLoop(0.0));

    // Turret
    getLeftStickLeft().whenActive(new TurretOpenLoop(0.3));  // Probalby 0.3
    getLeftStickRight().whenActive(new TurretOpenLoop(-0.3));  // Probalby -0.3
    getLeftStickOff().whenActive(new TurretOpenLoop(0.0));

    // Other
    // getRightDPAD().whenActive(new Hoodextend(true));
    // getRightDPAD().whenInactive(new Hoodextend (false));
    inPit.whenPressed(new InstantCommand(() -> {setRumble(RumbleType.kLeftRumble, 1.0);}));
    inPit.whenReleased(new InstantCommand(() -> {setRumble(RumbleType.kLeftRumble, 0.0);}));
    makeButton(Button.kStart).whenPressed(new InstantCommand(() -> {setRumble(RumbleType.kRightRumble, 1.0);}));
    makeButton(Button.kStart).whenReleased(new InstantCommand(() -> {setRumble(RumbleType.kRightRumble, 0.0);}));
  }

  public double getTurretJog() {
      return -operator.getRawAxis(XboxController.Axis.kLeftX.value);
  }

  public void setRumble(RumbleType side, double percent) {
    operator.setRumble(side, percent);
  }

  protected JoystickButton makeButton(Button button) {
    return new JoystickButton(operator, button.value);
  }

  protected Trigger getLeftStickLeft() {
    return new Trigger() {
      @Override
      public boolean get() { return operator.getRawAxis(XboxController.Axis.kLeftX.value) <= -LEFT_DEADBAND; }
    };
  }
  
  protected Trigger getLeftStickRight() {
    return new Trigger() {
      @Override
      public boolean get() { return operator.getRawAxis(XboxController.Axis.kLeftX.value) >= LEFT_DEADBAND; }
    };
  }

  protected Trigger getLeftStickOff() {
    return new Trigger() {
      @Override
      public boolean get() { return Math.abs(operator.getRawAxis(XboxController.Axis.kLeftX.value)) < LEFT_DEADBAND; }
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
}
