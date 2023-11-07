// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class DriveToObject extends CommandBase {
  private Drivetrain m_drivetrain;
  private Vision m_vision;
  private double speed = 0.3;
  private boolean doneTurning = false;
  private double angle_offset = 0;
  private double rotationvalue = 0;
  private int timer = 0;
  private boolean runOnce = false;
  private double range = 0.0;
  private PIDController turnController;

 

  

  /** Creates a new DriveToObject. */
  public DriveToObject(Drivetrain drivetrain, Vision vision) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    turnController = new PIDController(Constants.DrivetrainConstants.kPThetaController, 0, 0);
    addRequirements(m_drivetrain, m_vision);
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
    Rotation2d currentAngle = m_drivetrain.getNavxAngle();

    if (!m_vision.hasBestTarget()){
      System.out.println(timer);
      timer++;
      speed = 0;
    }
    else {
    timer = 0;
    if (!runOnce && m_vision.hasBestTarget()){
      angle_offset = m_vision.getBestObject().getYaw(); // not rly angleoffset but just angle placeholder
      runOnce = true;
    }
    
    if (!doneTurning){
      angle_offset = m_vision.getBestObject().getYaw();
      rotationvalue = -turnController.calculate(m_vision.getBestObject().getYaw(), 0);
      if (angle_offset < 2 && angle_offset > -2){
        doneTurning = true;
      }

      //m_drivetrain.turn(rotationvalue,angle_offset);
    }
    else{
    //System.out.println(m_vision.hasBestTarget());
    angle_offset =  currentAngle.getDegrees() - m_vision.getBestObject().getYaw();
     speed = 0.3;

    }


    

    m_drivetrain.driveMecanum(speed, speed, speed, speed);
  }
    m_drivetrain.turn(0.5,angle_offset);
    //m_drivetrain.driveMecanum(speed, speed, speed, speed);
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
    if(m_vision.hasBestTarget()){
      range = PhotonUtils.calculateDistanceToTargetMeters(0.508, 0.1651, Units.degreesToRadians(m_vision.getBestObject().getYaw()), Units.degreesToRadians(m_vision.getBestObject().getPitch()));
    
    if (timer/20 > 10)
      return true;
    
    
    return (range < 1);
    }
    return false; // if finished, return true or false IN METERS
}

//calculate Rotation
// private double calculateRotation() {
//   if (m_vision.hasBestTarget()) {
//     double targetYaw = m_vision.getBestObject().getYaw();
//     double currentYaw = m_drivetrain.getYaw();
    
//     // Calculate the desired rotation based on the difference in yaw angles.
//     double rotation = targetYaw - currentYaw;
    
//     return rotation;
//   } else {
//     return 0; // No rotation needed if no valid target is detected.
//   }
// }
}