package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase  {
    private final VictorSPX motor;
    public IntakeSubsystem() {
        motor = new VictorSPX(0);
    }
    public void setIntakeSpeed () {
        motor.set(ControlMode.PercentOutput, .5);
    }
}
