// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain;

public class DriveToObject extends CommandBase {
  private Drivetrain m_drivetrain;
  private Vision m_vision;
  private double speed = 0.3;
  private double angle_offset = 0;
  private int timer = 0;
  private boolean runOnce = false;

 

  

  /** Creates a new DriveToObject. */
  public DriveToObject(Drivetrain drivetrain, Vision vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vision.setPipeline(1); //whatever pipeline to use
      m_drivetrain.driveMecanum(0, 0, 0, 0);
    
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_vision.hasBestTarget()){
      System.out.println(timer);
      timer++;
    }
    else {
    System.out.println(m_vision.getObjectPosition().getX());
    timer = 0;
    if (!runOnce){
      angle_offset = m_drivetrain.getYaw() - m_vision.getBestObject().getYaw(); // angle offset of object to robot, need to turn to left until yaw is close to angle offset
      runOnce = true;
    }
    //System.out.println(m_vision.hasBestTarget());
    angle_offset =  m_vision.getObjectPosition().getY();
    m_drivetrain.driveMecanum(speed, speed, speed, speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vision.setPipeline(1);
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double range = PhotonUtils.calculateDistanceToTargetMeters(0.508, 0.1651, m_vision.getBestObject().getYaw(), m_vision.getBestObject().getPitch());
    if (timer/20 > 10)
      return true;
    
    
    return (range < 1); // if finished, return true or false IN METERS
}
}