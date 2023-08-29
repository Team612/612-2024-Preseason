// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.ctre.phoenixpro.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

/** Add your docs here. */

public class SwerveModule {
    private CANSparkMax drive_motor;
    private CANSparkMax angle_motor;
    private SparkMaxPIDController angle_controller;
    private SparkMaxPIDController drive_controller;
    private CANcoder angle_encoder; //this will be used as more of an accurate measurement compared to the built in Encoder
    private RelativeEncoder integrated_angle_encoder; //this will be used to actually adjust the position
    private int moduleNumber;
    private double velocity;
    //private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(null,null , null);  //used to calculate speeds with desire velocity and acceleration

    public SwerveModule(int mn, int port_motor,int port_angle, int port_encoder, MotorType t){
        moduleNumber = mn;
        drive_motor = new CANSparkMax(port_motor, t);
        angle_motor = new CANSparkMax(port_angle, t);
        angle_encoder = new CANcoder(port_encoder);
        integrated_angle_encoder = angle_motor.getEncoder();

        //creates PID controllers
        angle_controller = angle_motor.getPIDController();
        drive_controller = drive_motor.getPIDController();
        configureController();
    }

    public void configureController(){
        //TEMPORARY VALUES!!
        angle_controller.setP(0.0);
        angle_controller.setI(0.0);
        angle_controller.setD(0.0);
        angle_controller.setFF(0.0);

        drive_controller.setP(0.0);
        drive_controller.setI(0.0);
        drive_controller.setD(0.0);
        drive_controller.setFF(0.0);

    }

    public void configureEncoders(){
        integrated_angle_encoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
    }
    
    //as of right now, i am writing under the assumption that we will be using the CANCoders instead of the NEO's built in ones
    public void setDesiredState(SwerveModuleState state){
        setSpeed(state);
    }

    public void resetToAbsolute(){ //resets the module's wheel to 0 degrees
       integrated_angle_encoder.setPosition(0 - getRotations().getRotations());
    }

    public void setSpeed(SwerveModuleState state){
        velocity = state.speedMetersPerSecond / Constants.SwerveConstants.SwerveMaxSpeed;
        drive_motor.set(velocity); //.set is a percentage of speed, from -1 to 1
        //write pid stuff for this later
    }

    public void setAngle(SwerveModuleState state){ //switch to canENcoders bc neo ones arent accurate (42:1)
        Rotation2d angle = state.angle;
        angle_controller.setReference(angle.getDegrees(), ControlType.kPosition); //we're moving based off angular position, hence the control type
     } 

     public Rotation2d getRotations(){
        return Rotation2d.fromRotations(angle_encoder.getAbsolutePosition().getValue());
     }

     public Rotation2d getDegrees(){
        double angle = 360 * getRotations().getRotations();
        return Rotation2d.fromDegrees(angle);
     }

    public int getModuleNumber(){
        return moduleNumber;
    }
}
