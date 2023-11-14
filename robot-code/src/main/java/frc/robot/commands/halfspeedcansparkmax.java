// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.cansparkmax;

public class halfspeedcansparkmax extends CommandBase {
  /** Creates a new halfspeedcansparkmax. */
  private final cansparkmax m_cansparkmax;
  public halfspeedcansparkmax(cansparkmax cansparkmax) {
    m_cansparkmax = cansparkmax;
    addRequirements(m_cansparkmax);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cansparkmax.changeMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cansparkmax.changeMotorSpeed(m_cansparkmax.getMotorSpeed()+0.01);
    if (m_cansparkmax.getMotorSpeed() == 1) {
      m_cansparkmax.changeMotorSpeed(m_cansparkmax.getMotorSpeed()-0.01);
    } else if (m_cansparkmax.getMotorSpeed() == -1) {
      m_cansparkmax.changeMotorSpeed(m_cansparkmax.getMotorSpeed()+0.01);
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
