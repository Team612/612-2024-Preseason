// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.function.Supplier;
import java.util.stream.IntStream;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
public class Swerve extends SubsystemBase {

  static Swerve instance = null;
  private Translation2d fl;
  private Translation2d fr;
  private Translation2d bl;
  private Translation2d br;
  private SwerveModule[] SwerveModules = new SwerveModule[4];
  private SwerveDriveKinematics swerve_kinemtics;
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveModulePosition[] swerve_position;
  AHRS navx = new AHRS(I2C.Port.kMXP); //gyro
  /** Creates a new Swerve. */

  public Swerve(Translation2d fl,Translation2d fr,Translation2d bl,Translation2d br) {
    //Kinematics initalization
    this.fl = fl;
    this.fr = fr;
    this.bl = bl;
    this.br = br;

    swerve_kinemtics = new SwerveDriveKinematics(this.fl,this.fr,this.bl,this.br);


    //SwerveModule initalization
    SwerveModules[0] = new SwerveModule(1, Constants.DrivetrainConstants.SPARK_FL, Constants.DrivetrainConstants.SPARK_ANGLE_FL, Constants.DrivetrainConstants.ENCODER_ANGLE_FL,MotorType.kBrushless); //fl
    SwerveModules[1] = new SwerveModule(2, Constants.DrivetrainConstants.SPARK_FR, Constants.DrivetrainConstants.SPARK_ANGLE_FR,Constants.DrivetrainConstants.ENCODER_ANGLE_FR, MotorType.kBrushless); //fr
    SwerveModules[2] = new SwerveModule(3, Constants.DrivetrainConstants.SPARK_BL, Constants.DrivetrainConstants.SPARK_ANGLE_BL,Constants.DrivetrainConstants.ENCODER_ANGLE_BL, MotorType.kBrushless); //bl
    SwerveModules[3] = new SwerveModule(4, Constants.DrivetrainConstants.SPARK_BR, Constants.DrivetrainConstants.SPARK_ANGLE_BR,Constants.DrivetrainConstants.ENCODER_ANGLE_BR,  MotorType.kBrushless); //br

    swerve_position = new SwerveModulePosition[]{SwerveModules[0].getPositionObject(),SwerveModules[1].getPositionObject(),SwerveModules[2].getPositionObject(),SwerveModules[3].getPositionObject()};
    poseEstimator = new SwerveDrivePoseEstimator(swerve_kinemtics, getNavxAngle(),swerve_position , new Pose2d(0, 0, getNavxAngle()));
  }


  public void drive(double x, double y, double angle){ //Swerve comes with its own code for field relative pos
    ChassisSpeeds speed = new ChassisSpeeds(x,y,angle);
    SwerveModuleState[] moduleStates = swerve_kinemtics.toSwerveModuleStates(speed);

    for (SwerveModule mod : SwerveModules){
      mod.setDesiredState(moduleStates[mod.getModuleNumber()], true);
    }
    //swerve_kinemtics.desaturateWheelSpeeds(moduleStates, speed, x, x, y);
  


  }

  public Rotation2d getNavxAngle(){
    return Rotation2d.fromDegrees(navx.getAngle());
  }

  public Supplier<Pose2d> getRobotPosition(){
    Supplier<Pose2d> pose2d = () -> poseEstimator.getEstimatedPosition();
    return pose2d;
  }

  public SwerveDriveKinematics getKinematics(){
    return swerve_kinemtics;
  }
  public static Swerve getInstance(){
    if (instance == null){
      Constants.SwerveConstants sc = new Constants.SwerveConstants();
      instance = new Swerve(
        new Translation2d(sc.fl_distance[0],sc.fl_distance[1]),
        new Translation2d(sc.fr_distance[0],sc.fr_distance[1]),
        new Translation2d(sc.bl_distance[0],sc.bl_distance[1]),
        new Translation2d(sc.br_distance[0],sc.br_distance[1]));
    }
    return instance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
