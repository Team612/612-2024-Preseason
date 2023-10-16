// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
public class TurnTowardsObject extends CommandBase {
  Swerve m_swerve;
  Limelight m_limelight;
  /** Creates a new TurnTowardsObject. */
  public TurnTowardsObject(Swerve s, Limelight l) {
    m_swerve = s;
    m_limelight = l;
    addRequirements(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetWheelOrientation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("running");
    if (m_limelight.getTv()){
      double angle = m_limelight.getTx();
      m_swerve.drive(0, 0, angle);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_limelight.getTx()) <= 3){ //aprox
      return true;
    }
    return false;
  }
}
