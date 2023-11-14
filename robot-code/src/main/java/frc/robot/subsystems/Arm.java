// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.SparkPorts;
import frc.robot.controls.ControlMap;

public class Arm extends SubsystemBase {
  private static final double DEADZONE = 0.1;
  private CANSparkMax pivot;
  static Arm instance = null;
  
  /** Creates a new Arm. */
  public Arm() {
    pivot = new CANSparkMax(SparkPorts.pivotID, MotorType.kBrushless);
    pivot.setIdleMode(IdleMode.kBrake);
  }
  
  //rename method
  public void rotatePivot(double rotate) {
    
    if(Math.abs(rotate) < DEADZONE) rotate = 0;
    pivot.set(rotate);
  }

  public double getPivotEncoder() {
    return pivot.getEncoder().getPosition();
  }

  public Boolean withinThresh(){
    if(getPivotEncoder() > EncoderConstants.arm_lower && getPivotEncoder() < EncoderConstants.arm_upper){
      return true;
    }
    return false;
  }

  public static Arm getInstance(){
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  public boolean getPivotBottomLimitSwitchState(){
    return pivot.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    if (getPivotBottomLimitSwitchState()) {
      pivot.getEncoder().setPosition(0);
    }
  }
}
