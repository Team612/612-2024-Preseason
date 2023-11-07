// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
public class TurnToObject extends CommandBase {
  private Drivetrain m_drivetrain;
  private Vision m_vision;
  private PIDController turnController;
  private double obj_angle;
  private double timer;
  private double rotationspeed;
  /** Creates a new TurnToObject. */
  public TurnToObject(Drivetrain d, Vision v) {
    m_drivetrain = d;
    m_vision = v;
    turnController = new PIDController(Constants.DrivetrainConstants.kPThetaController, 0, 0);
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
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
      timer++;
      rotationspeed = 0;
    }
    else {
      timer = 0;
      obj_angle = m_vision.getBestObject().getYaw();
      rotationspeed = -turnController.calculate(obj_angle,0); //constantly calculates
    }
    
    m_drivetrain.turn(rotationspeed);
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
    if (m_vision.getBestObject().getYaw() < 2 && m_vision.getBestObject().getYaw() > -2){ //threshold of +-2
      return true;
    }
    if (timer/20 > 10)
      return true;

    return false;
  }
}
