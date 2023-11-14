// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  DoubleSolenoid exampleDoublePCM;
  DoubleSolenoid exampleDoublePH;
  public Intake() {
    exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    exampleDoublePH = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 4, 5);
  }
  public void setSolenoidStatus(int solenoidStatus) {
    if (solenoidStatus == 0) {
      exampleDoublePCM.set(kOff);
    } else if (solenoidStatus == 1) {
      exampleDoublePCM.set(kForward);
    } else if (solenoidStatus == 2) {
      exampleDoublePCM.set(kReverse);
    } else {
      for (var i = 3; i <= solenoidStatus; i++) {
        exampleDoublePCM.toggle();
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
