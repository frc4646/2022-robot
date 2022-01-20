package frc.robot.controls;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.TestAuto;

public class AutoModeSelector {
  enum StartingPosition {
    LEFT, RIGHT, CENTER
  }
  enum DesiredMode { 
    DO_NOTHING,
    TEST_AUTO,
  }

  private SendableChooser<DesiredMode> modeSelector;
  private SendableChooser<StartingPosition> startingPositionSelector;

  private DesiredMode modeDesiredCached = DesiredMode.DO_NOTHING;
  private StartingPosition startingPositionCached = null;

  private Optional<Command> mode = Optional.empty();

  public AutoModeSelector() {
    startingPositionSelector = new SendableChooser<>();
    startingPositionSelector.setDefaultOption("Left", StartingPosition.LEFT);
    startingPositionSelector.addOption("Right", StartingPosition.RIGHT);
    startingPositionSelector.addOption("Center", StartingPosition.CENTER);
    SmartDashboard.putData("Starting Position", startingPositionSelector);

    modeSelector = new SendableChooser<>();
    modeSelector.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
    modeSelector.addOption("TODO an actual auto mode", DesiredMode.TEST_AUTO);
    SmartDashboard.putData("Auto mode", modeSelector);
  }

  public void update() {
    DesiredMode desiredMode = modeSelector.getSelected();
    StartingPosition startingPosition = startingPositionSelector.getSelected();

    if (desiredMode == null) {
      desiredMode = DesiredMode.DO_NOTHING;
    }
    if (startingPosition == null) {
      startingPosition = StartingPosition.LEFT;
    }

    if (modeDesiredCached != desiredMode || startingPosition != startingPositionCached) {
        System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name() + ", starting position->" + startingPosition.name());
        mode = getAutoModeForParams(desiredMode, startingPosition);
    }
    modeDesiredCached = desiredMode;
    startingPositionCached = startingPosition;
  }

  private Optional<Command> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
    switch (mode) {
      case TEST_AUTO:
        return Optional.of(new TestAuto());
      default:
        break;
    }
    System.err.println("No valid auto mode found for  " + mode);
    return Optional.empty();
  }

  public void reset() {
    mode = Optional.empty();
    modeDesiredCached = null;
  }

  public void outputToSmartDashboard() {
    SmartDashboard.putString("AutoModeSelected", modeDesiredCached.name());
    SmartDashboard.putString("StartingPositionSelected", startingPositionCached.name());
  }

  public Optional<Command> getAutoMode() {
    return mode;
  }
}
