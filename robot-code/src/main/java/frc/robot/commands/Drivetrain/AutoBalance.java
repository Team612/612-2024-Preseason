// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalanceBETTER. */
  private final Drivetrain m_drivetrain;
  private double speed = 0.3;
  public int offbalancepositivehalf = 7; //might need to change later
  public AutoBalance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_drivetrain.getPitch() >= offbalancepositivehalf) {
      m_drivetrain.driveMecanum(speed, speed, speed, speed);
    } else if (m_drivetrain.getPitch() <= -offbalancepositivehalf) {
      m_drivetrain.driveMecanum(speed, speed, speed, speed);
    } else {
      m_drivetrain.driveMecanum(0, 0, 0, 0);
    }
    speed = m_drivetrain.getPitch() / 150;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}