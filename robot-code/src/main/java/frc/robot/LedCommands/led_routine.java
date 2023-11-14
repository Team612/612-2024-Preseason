// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LedCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;

public class led_routine extends CommandBase {
  /** Creates a new led_routine. */
  private final led m_led;
  private int counter;
  public led_routine(led led) {
    m_led = led;
    addRequirements(led);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter % 7 == 0) {
      m_led.red();
    } else if (counter % 7 == 1) {
      m_led.orange();
    } else if (counter % 7 == 2) {
      m_led.yellow();
    } else if (counter % 7 == 3) {
      m_led.green();
    } else if (counter % 7 == 4) {
      m_led.blue();
    } else if (counter % 7 == 5) {
      m_led.indigo();
    } else {
      m_led.violet();
    } counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter == 100;
  }
}
