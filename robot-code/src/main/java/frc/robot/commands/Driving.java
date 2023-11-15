// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Driving extends CommandBase {
  /** Creates a new LedTest. */
  private final Drivetrain m_drivetrain;
  private double distanceX;
  private double distanceY;
  private double x;
  private double y;
  private double distance = 0;


  public Driving(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain; 
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceX = m_drivetrain.getPose().getX();
    distanceY = m_drivetrain.getPose().getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = m_drivetrain.getPose().getX();
    y = m_drivetrain.getPose().getY();
    distance = Math.sqrt(Math.sqrt((distanceX-x)*(distanceX-x)+(distanceY-y)*(distanceY-y)));

    if ((distance)<12){
        m_drivetrain.driveMecanum(0.5, 0.5, 0.5, 0.5);
    }
    else {
      m_drivetrain.driveMecanum(0, 0, 0, 0);
      isFinished();
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
