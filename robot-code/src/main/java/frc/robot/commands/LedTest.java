// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedTest extends CommandBase {
  /** Creates a new LedTest. */
  private final led m_led;
  private int applePie = 0;
  private AddressableLEDBuffer m_ledBuffer;
  private int on = 1;


  public LedTest(led led) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_led = led; 
    m_ledBuffer = new AddressableLEDBuffer(300);
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    applePie = applePie + 1;
    // if (applePie % 2 == 1) {
    //   m_led.ChantillyTheme();
    // }
    // else {
    //   m_led.SixTwelveTheme();
    //   }


    // if (applePie<10){
    //   m_led.ChantillyTheme();
    // } else if (applePie<20){
    //   m_led.SixTwelveTheme();
    // } else {
    //   applePie = 0;
    // }

    

    if (on == 1){
      applePie = applePie + 1;
      if (applePie>200){
        on = 0;
      }
    }
    if (on == 0){
      applePie = applePie - 1;
      if (applePie<1){
        on = 1;
      }
    }
    m_ledBuffer.setRGB(1, 0, 0, applePie);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
