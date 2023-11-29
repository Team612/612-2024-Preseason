// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.Swerve;

public class FieldOrientedDrive extends CommandBase {
  /** Creates a new FieldOrientedDrive. */
  Swerve m_swerve;
  public FieldOrientedDrive(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetWheelOrientation();
    //m_drivetrain.setFodState(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_drivetrain.FieldOrientedDrive(-ControlMap.driver_joystick.getRawAxis(1), ControlMap.driver_joystick.getRawAxis(0), ControlMap.driver_joystick.getRawAxis(4));
    m_swerve.drive(-ControlMap.driver_joystick.getRawAxis(1), ControlMap.driver_joystick.getRawAxis(0), ControlMap.driver_joystick.getRawAxis(4));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
