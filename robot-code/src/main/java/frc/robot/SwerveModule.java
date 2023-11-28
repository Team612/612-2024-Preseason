// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private CANCoder module_encoder; //this will be used as more of an accurate measurement compared to the built in Encoder
    private RelativeEncoder integrated_angle_encoder; //this will be used to actually adjust the position
    private int moduleNumber;
    //private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(null,null , null);  //used to calculate speeds with desire velocity and acceleration
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.kS,
     Constants.SwerveConstants.kV,
      Constants.SwerveConstants.kA); 

    public SwerveModule(int n, int port_motor,int port_angle, int port_encoder, MotorType t){
        moduleNumber = n;
        drive_motor = new CANSparkMax(port_motor, t); //controls the speed of the robot
        angle_motor = new CANSparkMax(port_angle, t); //controls the angle of the robot
        module_encoder = new CANCoder(port_encoder); //encoder for the module
        integrated_angle_encoder = angle_motor.getEncoder(); //integrated encoder for the angle motor
        module_position = new SwerveModulePosition(); //keeps track of the modules angle and meters traveled

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

        drive_motor.setInverted(false);
        angle_motor.setInverted(false);
        drive_motor.burnFlash();
        angle_motor.burnFlash();

    }

    public void configureEncoders(){
        //integrated_angle_encoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor); //degrees per pulse
        drive_motor.getEncoder().setPositionConversionFactor(Constants.SwerveConstants.distancePerPulse);
        drive_motor.getEncoder().setVelocityConversionFactor(Constants.SwerveConstants.distancePerPulse / 60); //rps
        drive_motor.setSmartCurrentLimit(Constants.SwerveConstants.voltageLimits);
        angle_motor.setSmartCurrentLimit(Constants.SwerveConstants.voltageLimits);
        integrated_angle_encoder.setInverted(false);
        
        //saves settings

        
    }

    public void setDesiredState(SwerveModuleState state, boolean loop){ //the driving method.
        state = SwerveModuleState.optimize(state, getModuleAngle()); //minimizes travel time. If the robot is at 359 degrees, and needs to rotate to 0, its not going to rotate -359 degrees, but 1 degrees.
        setSpeed(state, loop);
        setAngle(state);
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

    public void setAngle(SwerveModuleState state){ 
        Rotation2d angle = state.angle; //kPositions takes in amount of rotations
        angle_controller.setReference(getRotationsFromDegree(angle), ControlType.kPosition); //we're moving based off angular position, hence the control type
     } 

     public void resetToAbsolute(){
        integrated_angle_encoder.setPosition(0);
        //module_encoder.setPosition(0);
     }
    

     public double getRotationsFromDegree(Rotation2d x){ //Assuming a 8.14:1 gear rato
        double angle = x.getDegrees() % 360; //relative angle
        if (angle < 0){ //to take shortest path back
            return Rotation2d.fromRotations(angle * -Constants.SwerveConstants.angleConversionFactor).getRotations(); //max amount of rotations backwards -16384.0
        }
        else {
            return Rotation2d.fromRotations(angle * Constants.SwerveConstants.angleConversionFactor).getRotations(); //max amount of rotations forward 16383.0
        }

        
     }
     //Assuming it takes 8.14 rotations of the Neo Motor for 1 rotation
     public Rotation2d getRawRotations(){
        return Rotation2d.fromRotations(integrated_angle_encoder.getPosition());
     }
     public Rotation2d getAbsoluteRotations(){ 
        return Rotation2d.fromRotations((integrated_angle_encoder.getPosition() % Constants.SwerveConstants.gearRatio)/ Constants.SwerveConstants.gearRatio);
     }


     public Rotation2d getModuleAngle(){
        double angle = module_encoder.getAbsolutePosition(); 
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
