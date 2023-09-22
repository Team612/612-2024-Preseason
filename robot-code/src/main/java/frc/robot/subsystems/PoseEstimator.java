// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  SwerveDrivePoseEstimator m_SwervePoseEstimator;
  PhotonPoseEstimator m_PhotonPoseEstimator;
  Vision m_Vision;
  Swerve s_swerve;
  private Field2d m_field;

  //values prob need to be edited later
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);
  private double previousPipelineTimestamp = 0;

  //Matrix Stds for state estimate
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.01);

  //Matrix Stds for vision estimates
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  static PoseEstimator estimator = null;
  

  public PoseEstimator() {
    s_swerve = Swerve.getInstance();
    m_Vision = Vision.getVisionInstance();
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);


    m_SwervePoseEstimator = new SwerveDrivePoseEstimator(
      s_swerve.getKinematics(), 
      s_swerve.getNavxAngle(), 
      s_swerve.getModulePosition(), 
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );
    m_PhotonPoseEstimator = m_Vision.getVisionPose();
  }

  public static PoseEstimator getPoseEstimatorInstance() {
    if (estimator == null) {
      estimator = new PoseEstimator();
    }
    return estimator;
  }

  private boolean once = true;


  @Override
  public void periodic() {

    if(Robot.initAllianceColor == Alliance.Blue && once){
      setPose(new Pose2d(0,0,new Rotation2d(Units.degreesToRadians(180))));
      once = false;
    }

    m_SwervePoseEstimator.update(s_swerve.getNavxAngle(), s_swerve.getModulePosition());
    if(m_PhotonPoseEstimator != null){
      m_PhotonPoseEstimator.update().ifPresent(estimatedRobotPose -> {
      var estimatedPose = estimatedRobotPose.estimatedPose;

      // Make sure we have a new measurement, and that it's on the field
      if (estimatedRobotPose.timestampSeconds != previousPipelineTimestamp && 
      estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
      && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {

        if (estimatedRobotPose.targetsUsed.size() > 1 && estimatedPose.getX() < 4) {

          for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {

            Pose3d targetPose = m_Vision.return_tag_pose(target.getFiducialId());
            Transform3d bestTarget = target.getBestCameraToTarget();
            Pose3d camPose = targetPose.transformBy(bestTarget.inverse());            
            double distance = Math.hypot(bestTarget.getX(), bestTarget.getY());

            //checking from the camera to the tag is less than 4
            if (distance < 4 && target.getPoseAmbiguity() <= .2) {
              previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
              m_SwervePoseEstimator.addVisionMeasurement(camPose.toPose2d(), estimatedRobotPose.timestampSeconds);
            }
          }
        } 

        else {
            previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
            m_SwervePoseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
        }
      }
      });
    }
    m_field.setRobotPose(getCurrentPose());
    m_SwervePoseEstimator.update(s_swerve.getNavxAngle(), s_swerve.getModulePosition());
  }


  public Pose2d getCurrentPose() {
    return m_SwervePoseEstimator.getEstimatedPosition();
  }

  public void resetPose() {
    m_SwervePoseEstimator.resetPosition(s_swerve.getNavxAngle(), s_swerve.getModulePosition(), getCurrentPose());
  }

  public void setPose(Pose2d newPose){
    m_SwervePoseEstimator.resetPosition(s_swerve.getNavxAngle(), s_swerve.getModulePosition(), newPose);
  }


    
  



}