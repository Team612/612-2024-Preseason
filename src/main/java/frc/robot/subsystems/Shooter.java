// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkMax prop; //propulsion motor
  double currentVelocity; // rotations per minute
  double maxVelocity = -21666; //re calculate 
  // private CANSparkMax hoodMotor = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax shooterLead = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless); //change ports later
  private CANSparkMax shooterFollow = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);   //motors 
  
   //PID Gains (need to update)
  double p, i, d, f ;
  //other stuff (idk what these mean)
  int iZone, slot;

  /** Creates a new Shooter. */
  public Shooter() {

    this.shooterLead.setInverted(true); // one moter goes one way one motor goes other way
    this.shooterFollow.setInverted(false); // we need to invert one of the motors so that the ball spins out

    // this.shooterLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // think these are for talons, not sparks?
    // this.shooterLead.configClosedloopRamp(0.25);
    // this.shooterLead.configMotionParameters(shooterMotionParameters);
  }

  public double getVelocity() { //siri
    return this.shooterLead.getSelectedSensorVelocity() / maxVelocity; /

  }

  public double getPosition() { //vik
    return this.shooterLead.getSelectedSensorPosition();
  }

  public double getCurrentDraw() {
    return this.shooterLead.getSupplyCurrent();
    //
  }

  public double getVoltageOutput() {  //shruthi
    return this.shooterLead.getBusVoltage();    //prob not right 
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Shooter Velocity", Math.abs(this.getVelocity()));
    // SmartDashboard.putNumber("Shooter Velocity Text",
    // Math.abs(this.getVelocity()));
    // SmartDashboard.putNumber("Current Draw", this.getCurrentDraw());
    // SmartDashboard.putNumber("Voltage Output",
    // Math.abs(this.getVoltageOutput()));
  }

  public void set(ControlMode controlMode, double setpoint) {
    shooterLead.set(controlMode, setpoint);
    shooterFollow.set(controlMode, setpoint);
  }

  public void setPercentVelocity(double percentVelocity) {
    this.set(ControlMode.Velocity, this.maxVelocity * percentVelocity);
  }

  // public void setHood(double setpoint) {
  // hoodMotor.set(setpoint);
  // } 
}
