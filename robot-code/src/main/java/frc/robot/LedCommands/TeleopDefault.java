// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.LedCommands;
import frc.robot.subsystems.led;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
public class TeleopDefault extends CommandBase {
  private led m_led;
  private Vision m_vision;
  /** Creates a new TeleopDefault. */
  public TeleopDefault(led l, Vision v) {
    m_led = l;
    m_vision = v;
    addRequirements(m_led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_led.setLed(255,0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.getCamera().getLatestResult().hasTargets()){
      m_led.setLed(0, 255, 0);
      // System.out.println("*******PRINTIIIIIIING****************");
    }else{
      if(Robot.initAllianceColor == Alliance.Blue) m_led.setLed(0, 0, 255);
      else if(Robot.initAllianceColor == Alliance.Red) m_led.setLed(255, 0, 0);
    
    }

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
