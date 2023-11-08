// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;

public class LedMed extends CommandBase {
  /** Creates a new LedMed. */
  private final led m_led;
  private final int woahs;
  // private final led m_ledBuffer;
  private int counter;
  public LedMed(led led, int woah) {
    woahs = woah;
    m_led = led;
    addRequirements(led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the comm2and is initially scheduled.
  @Override
  public void initialize() {
    m_led.resetled();
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      counter++;
      for (var i = 0; i < woahs; i++) {
        if(counter%2 == 0){
            m_led.setIndividalLed(i,255,0,0);
        }
        else{
            m_led.setIndividalLed(i,0,0,255);
        }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter == 10000000;
  }
}
