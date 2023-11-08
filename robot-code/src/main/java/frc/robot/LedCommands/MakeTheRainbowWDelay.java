// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;

public class MakeTheRainbowWDelay extends CommandBase {
  /** Creates a new LedMed. */
  private final led m_led;
//   private final int woahs;
  // private final led m_ledBuffer;
  private int counter;
  public MakeTheRainbowWDelay(led led, int woah) {
    // woahs = woah;
    m_led = led;
    addRequirements(led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.resetled();
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    if(counter % 50 == 0){
        m_led.CycleThroughTheRainbow();
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
