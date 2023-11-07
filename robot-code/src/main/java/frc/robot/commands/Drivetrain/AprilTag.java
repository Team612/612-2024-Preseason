// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;

public class AprilTag extends CommandBase {
  /** Creates a new AutoBalanceBETTER. */
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  private double speed = 0.3;
  public int offbalancepositivehalf = 7; //might need to change later
  public AprilTag(Drivetrain drivetrain, Vision vision) {
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vision.cameraresult(m_drivetrain);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
