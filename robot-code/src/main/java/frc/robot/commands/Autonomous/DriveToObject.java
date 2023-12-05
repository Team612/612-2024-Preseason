// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class DriveToObject extends CommandBase {
  private Drivetrain m_drivetrain;
  private Limelight m_limelight;
  private double speed = 0.3;
  private int timer = 0;
  private boolean runOnce = false;
  private double range = 18; //inches because we like inches
  private double kTargetHeight = 4;
  private double kMountHeight = 31.2;
  private double targetDistance = 60.0; // 60 inches = 5 feet cuz we like inches so too bad

 

  

  /** Creates a new DriveToObject. */
  public DriveToObject(Drivetrain drivetrain, Limelight lime) {
    m_drivetrain = drivetrain;
    m_limelight = lime;
    addRequirements(m_drivetrain, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_limelight.setPipeline(1); //whatever pipeline to use
      m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveMecanum(1, 1, 1 ,1);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setPipeline(1);
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if(m_limelight.getTv()){ 
      double dist = (kTargetHeight - kMountHeight) / (Math.tan(Units.degreesToRadians(m_limelight.getTy())) * Math.cos(Units.degreesToRadians(m_limelight.getTx()))); // need to change offset too
      if(dist < range){ // so this it very small threshold rn, will change later
        return true;
      } 
      
      if (timer/20 > 10) {  
        return true; 
      } 
      return false;
    }
    return false;
  }


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
//}