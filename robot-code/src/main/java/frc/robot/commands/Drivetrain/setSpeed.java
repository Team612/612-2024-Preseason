// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package main.java.frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.motorthingy;

public class SetSpeed extends CommandBase {
  /** Creates a new SetForward. */
  Drivetrain m_drivetrain;
  motrorthingy m_motorthingy;
  public SetForward(Drivetrain drivetrain,motorthingy motorthingy) {
    m_motorthingy = motorthingy;
    addRequirements(motorthingy);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially schedulead.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_motorthingy.settyTheSpeed(0.5);
    System.out.println(m_motorthingy.gettyTheSpeed());
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