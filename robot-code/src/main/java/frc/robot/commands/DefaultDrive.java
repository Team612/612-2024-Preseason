// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controls.ControlMap;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {
  private final Drivetrain m_drivetrain;

  public DefaultDrive(Drivetrain drivetrain) {
      m_drivetrain = drivetrain;
      addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
      m_drivetrain.drive(0.0, 0.0, 0.0);
  }

  @Override
  public void execute() {
      m_drivetrain.drive(-ControlMap.driver_joystick.getRawAxis(1), ControlMap.driver_joystick.getRawAxis(0), ControlMap.driver_joystick.getRawAxis(4));
  }

  @Override
  public void end(boolean interrupted) {
      m_drivetrain.drive(0.0, 0.0, 0.0);
  }
}
