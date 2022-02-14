package frc.robot.util;

import com.ctre.phoenix.led.Animation;

public class DiagnosticState {
  public final Animation diagnostic;
  public final boolean critical;

  public DiagnosticState(Animation diagnostic) {
    this(diagnostic, false);
  }
  
  public DiagnosticState(Animation diagnostic, boolean critical) {
    this.diagnostic = diagnostic;
    this.critical = critical;
  }
}
