// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
public class TurnTowardsObject extends CommandBase {

  private int timer = 0;
  private double rotationspeed = 0;

  Limelight m_limelight;
  Drivetrain m_drivetrain;
  private final double offset = 0; // NEEDS TO change depending on trials
  /** Creates a new TurnTowardsObject. */
  public TurnTowardsObject(Limelight l, Drivetrain m) {

    m_limelight = l;
    m_drivetrain = m;
    addRequirements(m_limelight);
    addRequirements(m_drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
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
    System.out.println("running");
    if (m_limelight.getTv()){
      double angle = m_limelight.getTx(); //need to add offset variable!
      
      if(obj_angle < 0){
        obj_angle = -obj_angle; // for like diff turning sides
      }

      timer = 0;
      rotationspeed = -turnController.calculate(obj_angle, 0); //same pid calculation, not affected by limelight i think


      System.out.println(rotationspeed); // printing these temporarily
      m_drivetrain.turn(rotationspeed);
      System.out.println(angle);

    }
  else{
      timer++;
      rotationspeed = 0;
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.turn(0);
    m_drivetrain.driveMecanum(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if(m_limelight.getTv()){ //siri go look fiscord i will send pic
      double yaw = m_limelight.getTx(); // need to change offset too
      if(yaw < (1+offset) && yaw > (-1 + offset)){ // so this it very small threshold rn, will change later
        return true;
      } 

      if (timer/20 > 10) {  // cuz timer is incremented in execute
        return true; 
      } 
      return false;
    }
}
