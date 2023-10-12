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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Add your docs here. */

public class SwerveModule {
    private CANSparkMax drive_motor;
    private CANSparkMax angle_motor;
    private SparkMaxPIDController angle_controller;
    private SparkMaxPIDController drive_controller;
    private SwerveModulePosition module_position;
    private CANcoder angle_encoder; //this will be used as more of an accurate measurement compared to the built in Encoder
    private RelativeEncoder integrated_angle_encoder; //this will be used to actually adjust the position
    private int moduleNumber;
    //private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(null,null , null);  //used to calculate speeds with desire velocity and acceleration
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.kS,
     Constants.SwerveConstants.kV,
      Constants.SwerveConstants.kA); 

    public SwerveModule(int n, int port_motor,int port_angle, int port_encoder, MotorType t){
        moduleNumber = n;
        drive_motor = new CANSparkMax(port_motor, t);
        angle_motor = new CANSparkMax(port_angle, t);
        angle_encoder = new CANcoder(port_encoder);
        module_position = new SwerveModulePosition(); //keeps track of the modules angle and meters traveled
        integrated_angle_encoder = angle_motor.getEncoder();

        //creates PID controllers
        angle_controller = angle_motor.getPIDController();
        drive_controller = drive_motor.getPIDController();
        configureController();
    }

    public void configureController(){
        //TEMPORARY VALUES!!
        angle_controller.setP(Constants.SwerveConstants.kP);
        angle_controller.setI(Constants.SwerveConstants.kI);
        angle_controller.setD(Constants.SwerveConstants.kD);
        angle_controller.setFF(0.0);

        drive_controller.setP(Constants.SwerveConstants.kP);
        drive_controller.setI(Constants.SwerveConstants.kI);
        drive_controller.setD(Constants.SwerveConstants.kD);
        drive_controller.setFF(0.0);

    }

    public void configureEncoders(){
        integrated_angle_encoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
        drive_motor.getEncoder().setPositionConversionFactor(Constants.SwerveConstants.distancePerPulse);
        drive_motor.getEncoder().setVelocityConversionFactor(Constants.SwerveConstants.distancePerPulse / 60); //rps
        drive_motor.getEncoder().setInverted(false);
        angle_motor.getEncoder().setInverted(false);
        integrated_angle_encoder.setInverted(false);
    }


    
    //as of right now, i am writing under the assumption that we will be using the CANCoders instead of the NEO's built in ones
    public void setDesiredState(SwerveModuleState state, boolean loop){
        setSpeed(state, loop);
        setAngle(state); 
    }

    public void resetToAbsolute(){ //resets the module's wheel to 0 degrees
       integrated_angle_encoder.setPosition(0 - getRawRotations().getRotations());
    }

    public void setSpeed(SwerveModuleState state, boolean isOpenLoop){
        if (isOpenLoop){ //if we are recieving continous input from the driver/controller. Anything that is not specific
        double velocity = state.speedMetersPerSecond / Constants.SwerveConstants.SwerveMaxSpeed;
        drive_motor.set(velocity); //.set is a percentage of speed, from -1 to 1
        }
        else { //gradual acceleration for autonomous things
            drive_controller.setReference(state.speedMetersPerSecond,ControlType.kVelocity,0, feedforward.calculate(state.speedMetersPerSecond)); 

        } 
  
    }
    //.setReference will automatically make the motor move by calling on the SmartMotion class... dont ask me how that works though
    public void setAngle(SwerveModuleState state){ 
        Rotation2d angle = state.angle; //kPositions takes in amount of rotations
        angle_controller.setReference(getRotationsFromDegree(angle), ControlType.kPosition); //we're moving based off angular position, hence the control type
     } 
    

     public double getRotationsFromDegree(Rotation2d x){
        double angle = x.getDegrees() / 360; //absolute position -0.5 - 0.99
        if (angle < 0){ //to take shortest path back
            return Rotation2d.fromRotations(angle * -16384.0).getRotations(); //difference from the reference
        }
        else {
            return Rotation2d.fromRotations(angle * 16383.0).getRotations(); //difference from the reference
        }

        
     }
     public Rotation2d getRawRotations(){
        return Rotation2d.fromRotations(angle_encoder.getPosition().getValue());
     }
     public Rotation2d getAbsoluteRotations(){ //absolute rotations will only count from -16384 - 16383
        return Rotation2d.fromRotations(angle_encoder.getAbsolutePosition().getValue());
     }

     public Rotation2d getDegrees(){
        double angle = 360 * getAbsoluteRotations().getRotations(); //gets the number of rotations (-0.5,0.99) and multiplies it by 360 to get current angle
        return Rotation2d.fromDegrees(angle);
     }

     public SwerveModulePosition getPositionObject(){
        return module_position;
     }

     public double getDistance(){
        return module_position.distanceMeters;
     }

    public int getModuleNumber(){
        return moduleNumber;
    }
}
