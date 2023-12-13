// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
public class Limelight extends SubsystemBase {
  private NetworkTable limelightTable;
  private Pose2d robotInTagPose;
  private static Limelight instance;
  private AprilTagFieldLayout aprilTagFieldLayout;
  /** Creates a new Limelight. */
  public Limelight() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    
        // tag 1
        final Translation3d translation1 = new Translation3d(15.513558, 1.071626, 0.462788);
        final Quaternion q1 = new Quaternion(0, 0, 0, 1);
        final Rotation3d rotation1 = new Rotation3d(q1);
        final AprilTag tag1 = new AprilTag(1, new Pose3d(translation1, rotation1));
    
        // tag 2
        final Translation3d translation2 = new Translation3d(15.513558, 2.748026, 0.462788);
        final Quaternion q2 = new Quaternion(0, 0, 0, 1);
        final Rotation3d rotation2 = new Rotation3d(q2);
        final AprilTag tag2 = new AprilTag(2, new Pose3d(translation2, rotation2));
    
        // tag 3
        final Translation3d translation3 = new Translation3d(15.513558, 4.424426, 0.462788);
        final Quaternion q3 = new Quaternion(0, 0, 0, 1);
        final Rotation3d rotation3 = new Rotation3d(q3);
        final AprilTag tag3 = new AprilTag(3, new Pose3d(translation3, rotation3));
    
        // tag 4
        final Translation3d translation4 = new Translation3d(16.178784, 6.749796, 0.695452);
        final Quaternion q4 = new Quaternion(0, 0, 0, 1);
        final Rotation3d rotation4 = new Rotation3d(q4);
        final AprilTag tag4 = new AprilTag(4, new Pose3d(translation4, rotation4));
    
        // tag 5
        final Translation3d translation5 = new Translation3d(0.36195, 6.749796, 0.695452);
        final Quaternion q5 = new Quaternion(1, 0, 0, 0);
        final Rotation3d rotation5 = new Rotation3d(q5);
        final AprilTag tag5 = new AprilTag(5, new Pose3d(translation5, rotation5));
    
        // tag 6
        final Translation3d translation6 = new Translation3d(1.02743, 4.424426, 0.462788);
        final Quaternion q6 = new Quaternion(1, 0, 0, 0);
        final Rotation3d rotation6 = new Rotation3d(q6);
        final AprilTag tag6 = new AprilTag(6, new Pose3d(translation6, rotation6));
    
        // tag 7
        final Translation3d translation7 = new Translation3d(1.02743, 2.748026, 0.462788);
        final Quaternion q7 = new Quaternion(1, 0, 0, 0);
        final Rotation3d rotation7 = new Rotation3d(q7);
        final AprilTag tag7 = new AprilTag(7, new Pose3d(translation7, rotation7));
    
        // tag 8
        final Translation3d translation8 = new Translation3d(1.02743, 1.071626, 0.462788);
        final Quaternion q8 = new Quaternion(1, 0, 0, 0);
        final Rotation3d rotation8 = new Rotation3d(q8);
        final AprilTag tag8 = new AprilTag(8, new Pose3d(translation8, rotation8));
        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        atList.add(tag1);
        atList.add(tag2);
        atList.add(tag3);
        atList.add(tag4);
        atList.add(tag5);
        atList.add(tag6);
        atList.add(tag7);
        atList.add(tag8);
    
        robotInTagPose = new Pose2d();
    
        aprilTagFieldLayout = new AprilTagFieldLayout(atList, 16.4592, 8.2296);
  }
  

  public void switchPipline(int n){ // 0 <= n <= 9
    limelightTable.getEntry("pipeline").setNumber(n);
  }


  //getters
  public double getTx(){ //horizontal deviation from the center. -27 - 27 degrees
    return limelightTable.getEntry("tx").getDouble(0.0);
  }

  public double getTy(){ //vertical deviation from the center. -27 - 27 degrees
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  public double getTa(){ //target area, 0% of the screen - 100% of the screen
    return limelightTable.getEntry("ta").getDouble(0.0);

  }

  public double getTagPose(int id){
    double[] array  = limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    return array[id];
  }

  public int getFiducialId(){
    return (int) limelightTable.getEntry("tid").getDouble(-1.0);
  }



  public boolean getTv(){ //valid targets true or false
    if (limelightTable.getEntry("tv").getDouble(0) == 1){
      return true;
    }
    return false;

  }

  



  public static Limelight getInstance(){
    if (instance == null){
      instance = new Limelight();
    }
    return instance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
