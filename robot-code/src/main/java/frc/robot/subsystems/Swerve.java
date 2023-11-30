// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.PoseEstimator;

import java.util.function.Supplier;
import java.util.stream.IntStream;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
public class Swerve extends SubsystemBase {

  static Swerve instance = null;
  private final SwerveModule[] SwerveModules;
  private SwerveDriveKinematics swerve_kinemtics;
  private PoseEstimator poseEstimator;
  private SwerveModulePosition[] swerve_position;
  private double angle_offset;
  private AHRS navx;
  /** Creates a new Swerve. */

  public Swerve() {
    //Kinematics initalization
    SwerveModules = new SwerveModule[4];
    navx = new AHRS(I2C.Port.kMXP); 
    swerve_kinemtics = Constants.SwerveConstants.swerve_kinemtics;

    
    //SwerveModule initalization
    //0 = fl
    //1 = fr
    //2 = bl
    //3 = br
    SwerveModules[0] = new SwerveModule(0, Constants.DrivetrainConstants.SPARK_FL, Constants.DrivetrainConstants.SPARK_ANGLE_FL, Constants.DrivetrainConstants.ENCODER_ANGLE_FL); //fl
    SwerveModules[1] = new SwerveModule(1, Constants.DrivetrainConstants.SPARK_FR, Constants.DrivetrainConstants.SPARK_ANGLE_FR, Constants.DrivetrainConstants.ENCODER_ANGLE_FR); //fr
    SwerveModules[2] = new SwerveModule(2, Constants.DrivetrainConstants.SPARK_BL, Constants.DrivetrainConstants.SPARK_ANGLE_BL, Constants.DrivetrainConstants.ENCODER_ANGLE_BL); //bl
    SwerveModules[3] = new SwerveModule(3, Constants.DrivetrainConstants.SPARK_BR, Constants.DrivetrainConstants.SPARK_ANGLE_BR,  Constants.DrivetrainConstants.ENCODER_ANGLE_BR); //br

    swerve_position = new SwerveModulePosition[]{SwerveModules[0].getPositionObject(),SwerveModules[1].getPositionObject(),SwerveModules[2].getPositionObject(),SwerveModules[3].getPositionObject()};
    //poseEstimator = new PoseEstimator();
  }

  public static Swerve getInstance(){
    if (instance == null){
      instance = new Swerve();
  }
    return instance;
  }

  public void drive(double x, double y, double angle){ //For the entire motor
    ChassisSpeeds speed = new ChassisSpeeds(x,y,angle);
    SwerveModuleState[] moduleStates = swerve_kinemtics.toSwerveModuleStates(speed);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,Constants.SwerveConstants.maxVelocityPerSecond); //so we dont go too fast
    for (SwerveModule mod : SwerveModules){
      mod.setDesiredState(moduleStates[mod.getModuleNumber()], true);
    }
  
  


  }

  

  public void drive(double x, double y, double angle, int motor){ //for individual motors
    ChassisSpeeds speed = new ChassisSpeeds(x,y,angle);
    SwerveModuleState[] moduleStates = swerve_kinemtics.toSwerveModuleStates(speed);
    for (SwerveModule mod : SwerveModules){
      if (mod.getModuleNumber() == motor){
        mod.setDesiredState(moduleStates[mod.getModuleNumber()], true);
      }
    }
  }

  public void resetWheelOrientation(){
    // ChassisSpeeds speed = new ChassisSpeeds(0,0,0);
    // SwerveModuleState[] moduleStates = swerve_kinemtics.toSwerveModuleStates(speed);
    for (SwerveModule mod : SwerveModules){
      //mod.setDesiredState(moduleStates[mod.getModuleNumber()], true);
      mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false);
      mod.resetToAbsolute();
    }
  }

  public Rotation2d getNavxAngle(){
    return Rotation2d.fromDegrees(navx.getAngle());
  }

  public void setAngleOffset(){
    angle_offset = SwerveModules[0].getModuleAngle().getDegrees(); //in theory, should be the same throughout
  }

  public double getVelocityX(){ //use for debugging later
    return navx.getVelocityX();
  }

  public double getVelocityY(){ //use for debugging later
    return navx.getVelocityY();
  }

  public PoseEstimator getPoseEstimator(){
    return poseEstimator;
  }

  public void setNavxAngleOffset(Rotation2d offset){
    navx.setAngleAdjustment(offset.getDegrees());
  }

  public Rotation2d getEstimatedRobotRotations(){
    return poseEstimator.getCurrentPose().getRotation();
  }

  public Rotation2d getModularRawRotations(int num){
    return SwerveModules[num].getRawRotations();
  }

  public Rotation2d getModularAbsoluteRotations(int num){
    return SwerveModules[num].getAbsoluteRotations();
  }

  public Supplier<Pose2d> getRobotPosition(){
    Supplier<Pose2d> pose2d = () -> poseEstimator.getCurrentPose();
    return pose2d;
  }

  public SwerveModulePosition[] getModulePosition(){
    return swerve_position;
  }

  



  public void resetPoseEstimator(){ 
    poseEstimator.resetPose();
    
  }
  

  public static SwerveControllerCommand runTrajectory(Swerve s_swerve, PathPlannerTrajectory path){
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.SwerveConstants.kPTheta,
     Constants.SwerveConstants.kI,
      Constants.SwerveConstants.kD,
       new TrapezoidProfile.Constraints(Constants.SwerveConstants.SwerveMaxSpeed, 5)
       );

    PIDController xController = new PIDController(Constants.SwerveConstants.kP,
    Constants.SwerveConstants.kI,
    Constants.SwerveConstants.kD); 
    PIDController yController = new PIDController(Constants.SwerveConstants.kP, 
    Constants.SwerveConstants.kI, 
    Constants.SwerveConstants.kD); 

    return new SwerveControllerCommand(path, s_swerve.getRobotPosition(),Constants.SwerveConstants.swerve_kinemtics, xController, yController,thetaController, null);
    
  }

  /*
     public void RobotOrientedDrive(double y, double x, double zRot){
    if(Math.abs(x) < DEADZONE) x = 0;
    if(Math.abs(y) < DEADZONE) y = 0;
    if(Math.abs(zRot) < DEADZONE) zRot = 0;
    drivetrain.driveCartesian(y, x, zRot);
  }

  //Field Oriented Drive

  public void FieldOrientedDrive(double x, double y, double zRotation){
    if(Math.abs(x) < DEADZONE) x = 0;
    if(Math.abs(y) < DEADZONE) y = 0;
    if(Math.abs(zRotation) < DEADZONE) zRotation = 0;
    drivetrain.driveCartesian(x, y, zRotation, getNavxAngle().unaryMinus().minus(navxAngleOffset.unaryMinus()));
    // System.out.println("******************************************" + navxAngleOffset + "**************************************");
  }
   */
  




  //public static CommandBase followTrajectory(){
    
  //}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
