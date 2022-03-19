package frc.robot.controls;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.ModeMiddle;
import frc.robot.commands.auto.ModeRight;
import frc.robot.commands.auto.ModeTest;
import frc.robot.commands.auto.ModeRight5Cargo;
import frc.robot.commands.auto.ModeFallback;
import frc.robot.commands.auto.ModeLeft;
import frc.robot.commands.auto.ModeBase.STRATEGY_PHASE_2;

public class AutoModeSelector {
  // enum StartingPosition {
  //   LEFT, MIDDLE, RIGHT
  // }
  // private SendableChooser<StartingPosition> startingPositionSelector;
  // private StartingPosition startingPositionCached = StartingPosition.LEFT;

  enum DesiredMode {
    DO_NOTHING,
    FALLBACK_2_CARGO,
    MIDDLE_4_CARGO,
    LEFT_2_CARGO,
    RIGHT_2_CARGO,
    TEST_1,
    TEST_2,
    RIGHT_5_CARGO,
  }

  private SendableChooser<DesiredMode> modeSelector;
  private DesiredMode modeDesiredCached = DesiredMode.DO_NOTHING;
  private Optional<Command> mode = Optional.empty();

  public AutoModeSelector() {
    // startingPositionSelector = new SendableChooser<>();
    // startingPositionSelector.setDefaultOption("Left", StartingPosition.LEFT);
    // startingPositionSelector.addOption("Middle", StartingPosition.MIDDLE);
    // startingPositionSelector.addOption("Right", StartingPosition.RIGHT);
    // SmartDashboard.putData("Auto: Position", startingPositionSelector);

    modeSelector = new SendableChooser<>();
    modeSelector.setDefaultOption("Fallback 2 Cargo", DesiredMode.FALLBACK_2_CARGO);
    modeSelector.addOption("Middle 4 Cargo", DesiredMode.MIDDLE_4_CARGO);
    modeSelector.addOption("Right 5 Cargo", DesiredMode.RIGHT_5_CARGO);
    modeSelector.addOption("Left 2 Cargo", DesiredMode.LEFT_2_CARGO);
    modeSelector.addOption("Right 2 Cargo", DesiredMode.RIGHT_2_CARGO);
    modeSelector.addOption("Do Nothing", DesiredMode.DO_NOTHING);
    modeSelector.addOption("Test 1", DesiredMode.TEST_1);
    modeSelector.addOption("Test 2", DesiredMode.TEST_2);
    SmartDashboard.putData("Auto: Mode", modeSelector);
  }

  private Optional<Command> getAutoModeForParams(DesiredMode mode) {
    switch (mode) {
      case MIDDLE_4_CARGO:
        return Optional.of(new ModeMiddle(STRATEGY_PHASE_2.HUMAN_PLAYER));
      case LEFT_2_CARGO:
        return Optional.of(new ModeLeft(STRATEGY_PHASE_2.NONE));
      case RIGHT_5_CARGO:
        return Optional.of(new ModeRight5Cargo());
      case RIGHT_2_CARGO:
        return Optional.of(new ModeRight(STRATEGY_PHASE_2.NONE));
      case TEST_1:
        return Optional.of(new ModeTest(false));
      case TEST_2:
        return Optional.of(new ModeTest(true));
      case DO_NOTHING:
        return Optional.of(new WaitCommand(15.0));
      case FALLBACK_2_CARGO:
      default:
        System.err.println("No valid auto mode found for  " + mode);
        return Optional.of(new ModeFallback());
    }
  }  

  public void update() {
    DesiredMode desiredMode = modeSelector.getSelected();

    if (desiredMode == null) {
      desiredMode = DesiredMode.FALLBACK_2_CARGO;
    }

    if (modeDesiredCached != desiredMode) {
      System.out.println("Auto: " + desiredMode.name());
      mode = getAutoModeForParams(desiredMode);
    }
    modeDesiredCached = desiredMode;
  }

  public void reset() {
    mode = Optional.empty();
    modeDesiredCached = null;
  }

  public Optional<Command> getAutoMode() {
    return mode;
  }
}
