package frc.robot.util;

public class DiagnosticState {
  public final LEDColor color;
  public final boolean critical;

  public DiagnosticState(LEDColor diagnostic) {
    this(diagnostic, false);
  }
  
  public DiagnosticState(LEDColor diagnostic, boolean critical) {
    this.color = diagnostic;
    this.critical = critical;
  }
}
