// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
// import frc.robot.Constants;
import frc.robot.Constants;
public class Vision extends SubsystemBase {
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private static Transform3d robotToCam;
  public PhotonPoseEstimator m_PoseEstimator;

  static Vision visionInstance = null;
  
  PhotonCamera camera;

  private Pose2d robotInTagPose;
  /**
   * Creates a new Vision.
   * 
   * @throws IOException
   **/

  public Vision(PhotonCamera camera) {
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
    this.camera = camera;
    resetRobotPose();

    aprilTagFieldLayout = new AprilTagFieldLayout(atList, 16.4592, 8.2296);
    // robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(1), Units.inchesToMeters(ShuffleBoardButtons.transformationYTune.getDouble(0)), 
    //              Units.inchesToMeters(0)), new Rotation3d(0,0,0));
    robotToCam = new Transform3d();
    
    m_PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, this.camera,
        robotToCam);
    m_PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


  }

  public static Vision getVisionInstance() {
    if (visionInstance == null) {
      visionInstance = new Vision(new PhotonCamera(Constants.VisionConstants.cameraName));
    }
    return visionInstance;
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  // getting the vision pose from the april tags
  public Pose2d getTagPose() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();

      Transform3d tagSpace = bestTarget.getBestCameraToTarget();

      return new Pose2d(tagSpace.getX(), tagSpace.getY(), new Rotation2d( (bestTarget.getYaw()) * (Math.PI/180)) );
    }
    return new Pose2d();
  }

    //set pipeline
    //pre: 0-9
  public void setPipeline(Integer pipeline) {
    if(pipeline<0){
        pipeline = 0;
    }else if(pipeline>9){
        pipeline = 9;
    }
    camera.setPipelineIndex(pipeline);
}

  public boolean hasBestTarget(){
    if (camera.getLatestResult().hasTargets())
      return true;
    return false;


  }

  public Pose2d getRobotPose(){
    Pose2d tagPose = robotInTagPose;
    return new Pose2d().transformBy(new Transform2d(tagPose.getTranslation(), tagPose.getRotation()));
  }
  
  
  public Transform3d getObjectPosition(){
    return camera.getLatestResult().getBestTarget().getBestCameraToTarget();
  }

  public PhotonTrackedTarget getBestObject(){
    return camera.getLatestResult().getBestTarget();
  }

  public void resetRobotPose(){
    robotInTagPose = getTagPose();
  }

  // return the tag pose
  public Pose3d return_tag_pose(int id) {
    Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
    return pose_of_tag.get();
  }

  // self calculations
  public Pose3d return_camera_pose_tag(int id, PhotonPipelineResult results) {
    Optional<Pose3d> pose_of_tag = aprilTagFieldLayout.getTagPose(id);
    Pose3d tag_pose = pose_of_tag.get();
    Transform3d cameraTransform = results.getBestTarget().getBestCameraToTarget();
    return tag_pose.plus(cameraTransform);
  }

  // photonvision pose estimator
  public Optional<EstimatedRobotPose> return_photon_pose(Pose2d latestPose) {
    m_PoseEstimator.setReferencePose(latestPose);
    return m_PoseEstimator.update();
  }

  public PhotonPoseEstimator getVisionPose(){
    return m_PoseEstimator;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(getRobotPose());
  }
}