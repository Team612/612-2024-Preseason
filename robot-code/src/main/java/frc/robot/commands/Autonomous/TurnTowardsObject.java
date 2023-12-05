// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// public class TurnTowardsObject extends CommandBase {
//   private final ProfiledPIDController turnController = Constants.VisionConstants.rotationController;
//   private int timer = 0;
//   private double rotationspeed = 0;
//   private final double offset = 0.0;
//   Limelight m_limelight;
//   Drivetrain m_drivetrain;
//   // NEEDS TO change depending on trials
  
//   /** Creates a new TurnTowardsObject. */
//   public TurnTowardsObject(Drivetrain drive, Limelight lime) {
    
//     m_limelight = lime;
//     m_drivetrain = drive;
//     addRequirements(m_limelight);
//     addRequirements(m_drivetrain);

//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_limelight.setPipeline(0); //whatever pipeline to use
//     m_drivetrain.driveMecanum(0, 0, 0, 0);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     System.out.println("running");
//     if (m_limelight.getTv()){
//       double angle = m_limelight.getTx(); //need to add offset variable!
      
//       if(angle < 0){
//         angle = -angle; // for like diff turning sides
//       }

//       timer = 0;
//       rotationspeed = -turnController.calculate(angle, 0); //same pid calculation, not affected by limelight i think


//       System.out.println(rotationspeed); // printing these temporarily
//       m_drivetrain.turn(rotationspeed);
//       System.out.println(angle);

//     }
//   else{
//       timer++;
//       rotationspeed = 0;
//   }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_drivetrain.turn(0);
//     m_drivetrain.driveMecanum(0, 0, 0, 0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() { 
//     if(m_limelight.getTv()){ //siri go look fiscord i will send pic
//       double yaw = m_limelight.getTx(); // need to change offset too
//       if(yaw < (1+offset) && yaw > (-1 + offset)){ // so this it very small threshold rn, will change later
//         return true;
//       } 

//       if (timer/20 > 10) {  // cuz timer is incremented in execute
//         return true; 
//       } 
//       return false;
//     }
//     return false;
//   }
// }
public class TurnTowardsObject extends CommandBase {
  private final ProfiledPIDController turnController = Constants.VisionConstants.rotationController;
  private int timer = 0;
  private double rotationspeed = 0;
  private final double offset = 0.0; // Adjust this offset as needed
  private final Limelight m_limelight;
  private final Drivetrain m_drivetrain;
  public TurnTowardsObject(Drivetrain drive, Limelight lime) {
      m_limelight = lime;
      m_drivetrain = drive;
      addRequirements(m_limelight, m_drivetrain);
  }
  @Override
  public void initialize() {
      m_limelight.setPipeline(1); // Set the pipeline
      m_drivetrain.driveMecanum(0, 0, 0, 0);
  }
  @Override
  public void execute() {
      if (m_limelight.getTv()) {
          double angle = m_limelight.getTx() + offset; // Consider applying the offset
          timer = 0;
          rotationspeed = -turnController.calculate(angle, 0);
          m_drivetrain.turn(rotationspeed);
      } else {
          timer++;
          rotationspeed = 0;
      }
  }
  @Override
  public void end(boolean interrupted) {
      m_drivetrain.turn(0);
      m_drivetrain.driveMecanum(0, 0, 0, 0);
  }
  @Override
  public boolean isFinished() {
      if (m_limelight.getTv()) {
          double yaw = m_limelight.getTx() + offset;
          return Math.abs(yaw) < 1; // Check within a small threshold
      }
      return timer / 20 > 10; // Check for a timer limit
  }
}









