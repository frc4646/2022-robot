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
  private StartingPosition startingPositionCached = StartingPosition.LEFT;

  private Optional<Command> mode = Optional.empty();

  public AutoModeSelector() {
    startingPositionSelector = new SendableChooser<>();
    startingPositionSelector.setDefaultOption("Left", StartingPosition.LEFT);
    startingPositionSelector.addOption("Right", StartingPosition.RIGHT);
    startingPositionSelector.addOption("Center", StartingPosition.CENTER);
    SmartDashboard.putData("Starting Position", startingPositionSelector);

    modeSelector = new SendableChooser<>();
    modeSelector.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
    modeSelector.addOption("Test Mode", DesiredMode.TEST_AUTO);
    SmartDashboard.putData("Auto Mode", modeSelector);
  }

  public void update() {
    DesiredMode desiredMode = modeSelector.getSelected();
    StartingPosition startingPosition = startingPositionSelector.getSelected();

    if (desiredMode == null || startingPosition == null) {
      desiredMode = DesiredMode.DO_NOTHING;
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
        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }
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
