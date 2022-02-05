package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.Constants;

public class Pneumatics extends SmartSubsystem {

  PneumaticsControlModule pcm;
  public Pneumatics() {
    pcm = new PneumaticsControlModule(Constants.CAN.PNEUMATIC_CONTROL_MODULE);

  }

}
