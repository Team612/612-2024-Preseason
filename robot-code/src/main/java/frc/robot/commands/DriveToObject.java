// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;

public class DriveToObject extends CommandBase {
  private Drivetrain m_drivetrain;
  private Vision m_vision;
  private double speed = 0.3;

  /** Creates a new DriveToObject. */
  public DriveToObject(Drivetrain drivetrain, Vision vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angle_offset = m_drivetrain.getYaw() - m_vision.getBestObject().getYaw(); // angle offset of object to robot, need to turn to left until yaw is close to angle offset
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle_offset =  m_vision.getObjectPosition().getY();
    m_drivetrain.driveMecanum(speed, speed, speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_vision.getObjectPosition().getX() < 5 ); // if finished, return true or false
}
}