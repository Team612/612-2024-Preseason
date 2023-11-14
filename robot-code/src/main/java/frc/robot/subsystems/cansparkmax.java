// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.lang.AutoCloseable;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class cansparkmax extends SubsystemBase {
  /** Creates a new cansparkmax. */
  private CANSparkMax m_canSparkMotor;
  public cansparkmax() {
    m_canSparkMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
  }
  public void changeMotorSpeed(double motorSpeed) {
    if ((-1 < motorSpeed) && (motorSpeed < 1)) {
      m_canSparkMotor.set(motorSpeed);
    } else if (motorSpeed <= -1) {
      m_canSparkMotor.set(-1);
    } else {
      m_canSparkMotor.set(1);
    }
    
  }
  public double getMotorSpeed() {
    return m_canSparkMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
