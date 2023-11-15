// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import Compressor;

// public class intake extends SubsystemBase {
//   ///** Creates a new intake. */
//   public intake() {
//     Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
//     Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
//   }

//   @Override
//   public void periodic() {
//     // //This method will be called once per scheduler run
//   }
// }



// pcmCompressor.enableDigital();
// pcmCompressor.disable();

// boolean enabled = pcmCompressor.enabled();
// boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
// double current = pcmCompressor.getCompressorCurrent();