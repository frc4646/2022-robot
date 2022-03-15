package frc.robot.controls;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.ModeMiddle;
import frc.robot.commands.auto.TestAutoPathweaver;
import frc.robot.commands.auto.FallbackTwoCargoAuto;
import frc.robot.commands.auto.ModeBase.STRATEGY_PHASE_2;

public class AutoModeSelector {
  enum StartingPosition {
    LEFT, MIDDLE, RIGHT
  }
  enum DesiredMode { 
    DO_NOTHING,
    TEST_AUTO,
    TWO_CARGO,
    TWO_CARGO_PLUS_TERMINAL,
    TWO_CARGO_PLUS_TERMINAL_PATHWEAVER,
  }

  private SendableChooser<DesiredMode> modeSelector;
  private SendableChooser<StartingPosition> startingPositionSelector;

  private DesiredMode modeDesiredCached = DesiredMode.DO_NOTHING;
  private StartingPosition startingPositionCached = StartingPosition.LEFT;

  private Optional<Command> mode = Optional.empty();

  public AutoModeSelector() {
    startingPositionSelector = new SendableChooser<>();
    startingPositionSelector.setDefaultOption("Left", StartingPosition.LEFT);
    startingPositionSelector.addOption("Middle", StartingPosition.MIDDLE);
    startingPositionSelector.addOption("Right", StartingPosition.RIGHT);
    SmartDashboard.putData("Auto: Position", startingPositionSelector);

    modeSelector = new SendableChooser<>();
    modeSelector.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
    modeSelector.addOption("2 Cargo", DesiredMode.TWO_CARGO);
    modeSelector.addOption("2 Cargo + Terminal", DesiredMode.TWO_CARGO_PLUS_TERMINAL);
    modeSelector.addOption("2 Cargo + Terminal Pathweaver", DesiredMode.TWO_CARGO_PLUS_TERMINAL_PATHWEAVER);
    modeSelector.addOption("Test Mode", DesiredMode.TEST_AUTO);
    SmartDashboard.putData("Auto: Mode", modeSelector);
  }

  private Optional<Command> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
    switch (mode) {
      case TWO_CARGO:
        return Optional.of(new FallbackTwoCargoAuto());

      case TWO_CARGO_PLUS_TERMINAL:
        switch (position) {
          case LEFT:
            return Optional.of(new FallbackTwoCargoAuto());
          case MIDDLE:
            return Optional.of(new ModeMiddle(STRATEGY_PHASE_2.HUMAN_PLAYER));
          case RIGHT:
            return Optional.of(new FallbackTwoCargoAuto());
        }

      case TWO_CARGO_PLUS_TERMINAL_PATHWEAVER:
        switch (position) {
          case LEFT:
            return Optional.of(new FallbackTwoCargoAuto());
          case MIDDLE:
            return Optional.of(new TestAutoPathweaver());
          case RIGHT:
            return Optional.of(new FallbackTwoCargoAuto());
        }

      case TEST_AUTO:
        return Optional.of(new ModeMiddle(STRATEGY_PHASE_2.HUMAN_PLAYER));

      default:
        System.err.println("No valid auto mode found for  " + mode);
        return Optional.of(new WaitCommand(15.0));
    }
  }  

  public void update() {
    DesiredMode desiredMode = modeSelector.getSelected();
    StartingPosition startingPosition = startingPositionSelector.getSelected();

    if (desiredMode == null || startingPosition == null) {
      desiredMode = DesiredMode.DO_NOTHING;
      startingPosition = StartingPosition.LEFT;
    }

    if (modeDesiredCached != desiredMode || startingPosition != startingPositionCached) {
      System.out.println("Auto: Mode->" + desiredMode.name() + ", Position->" + startingPosition.name());
      mode = getAutoModeForParams(desiredMode, startingPosition);
    }
    modeDesiredCached = desiredMode;
    startingPositionCached = startingPosition;
  }

  public void reset() {
    mode = Optional.empty();
    modeDesiredCached = null;
    startingPositionCached = null;
  }

  public Optional<Command> getAutoMode() {
    return mode;
  }
}
