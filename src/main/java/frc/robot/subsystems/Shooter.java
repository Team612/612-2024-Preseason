// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  double currentVelocity; // rotations per minute
  double maxVelocity = -21666; //re calculate 
  // private CANSparkMax hoodMotor = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax leftprop = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless); //propulsion motor
  private CANSparkMax rightprop = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);  // second propulsion motor

  private final PIDController velocityController = new PIDController(0.1, 0.0, 0.0); // Pid used to contorl velocity
  
   //PID Gains (need to update)
  double p, i, d, f ;
  //other stuff (idk what these mean)
  int iZone, slot;

  /** Creates a new Shooter. */
  public Shooter() {

    leftprop.setInverted(true); // one moter goes one way one motor goes other way
    rightprop.setInverted(false); // we need to invert one of the motors so that the ball spins out

    // this.shooterLead.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // think these are for talons, not sparks?
    // this.shooterLead.configClosedloopRamp(0.25);
    // this.shooterLead.configMotionParameters(shooterMotionParameters);

    velocityController.setTolerance(100); // adjust value based on requirements
  }

  public double getVelocity() { //siri
    return leftprop.getEncoder().getVelocity() / maxVelocity; /

  }

  // for accurate shooting
  public void setVelocity(double targetVelocity) {
    // calulates how much power needs to be given to shooter motors in order to reach desired speed
    double motorOutput = velocityController.calculate(getVelocity(), targetVelocity) / MAX_VELOCITY;

    // Actually setting the motors to reach desired speed after calculation using pid
    leftMotor.set(motorOutput);
    rightMotor.set(motorOutput);
}

  public double getPosition() { //vik
    return leftprop.getEncoder().getSelectedSensorPosition();
  }

  public double getCurrentDraw() {
    return leftprop.getSupplyCurrent();
    //
  }

  public double getVoltageOutput() {  //shruthi
    return leftprop.getBusVoltage();    //prob not right 
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

  public void set(double value) {
    // value has to be within -1 and 1
    leftprop.set(value);
    rightprop.set(value);
  }

  public void setPercentVelocity(double percentVelocity) {
    //calculates desired velocity as a percent/portion of the maximum velocity
    //set motor velocities based on this value
    this.set(this.maxVelocity * percentVelocity);
  }

  // public void setHood(double setpoint) {
  // hoodMotor.set(setpoint);
  // } 
}
