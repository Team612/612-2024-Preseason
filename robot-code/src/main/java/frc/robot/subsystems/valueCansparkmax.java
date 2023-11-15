// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import com.revrobotics.SparkMaxLimitSwitch;

// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.EncoderConstants;
// import frc.robot.Constants.SparkPorts;
// import frc.robot.controls.ControlMap;
//use rev

public class valueCansparkmax extends SubsystemBase {
  private CANSparkMax CSM;
  private double thing;
  /** Creates a new valueCansparkmax. */
  public valueCansparkmax() {
    CSM = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    CSM.setIdleMode(IdleMode.kBrake);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getvalue() {
    thing = CSM.get();
    return thing;
  }

  public void setvalue(double speed) {
    CSM.set(speed);
  }

}
