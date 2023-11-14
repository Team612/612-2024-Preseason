// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Unfinished
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {
    Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
    pcmCompressor.enableDigital();
    phCompressor.disable();
    boolean enabled = pcmCompressor.enabled();
    boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
