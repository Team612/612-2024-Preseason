
package frc.robot.commands.Drivetrain;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.ShuffleBoardButtons;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class TrajectoryCreation {

    public TrajectoryConfig config = new TrajectoryConfig(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq)
        .setKinematics(Constants.SwerveConstants.swerve_kinemtics);

    public TrajectoryConfig config_backwards = new TrajectoryConfig(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq)
        .setKinematics(Constants.SwerveConstants.swerve_kinemtics).setReversed(true);
    

    public Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(2,0), new Translation2d(2,-2), new Translation2d(0, -2)),
        new Pose2d(0,0, new Rotation2d(0)), 
        config); 
    
    public Trajectory StrafeRightMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();
        
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x,y-0.5)),
            new Pose2d(x,y-1, new Rotation2d(Units.degreesToRadians(degrees))),
            config_backwards
        );
    }

    public Trajectory StrafeLeftMeter(PoseEstimator estimation){ 
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();
        
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x,y+0.5)),
            new Pose2d(x,y+1, new Rotation2d(Units.degreesToRadians(degrees))),
            config
        );
    }

    public Trajectory ForwardMeter(PoseEstimator estimation){

        //figure out a way to find an initial position 

        // System.out.println("*************************** PRINT **********************************");
        // var currentPose = estimation.getCurrentPose();
        // System.out.println(currentPose);
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();

        

        // System.out.println("*************************** END PRINT **********************************");

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x + 0.5,y)),
            new Pose2d(x + 1.0, y, new Rotation2d(degrees)),
            config
        );
    }

    public Trajectory BackwardMeter(PoseEstimator estimation){
        Pose2d estimatedPose = estimation.getCurrentPose();
        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        double degrees = estimatedPose.getRotation().getRadians();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(x, y, new Rotation2d(degrees)),
            List.of(new Translation2d(x-2.5,y)),
            new Pose2d(x-4.75, y, new Rotation2d(degrees
            )),
            config_backwards
        );
    }

    public PathPlannerTrajectory onthefly(PoseEstimator estimator, Vision vision, double y_translation){
        Pose2d estimatedPose = estimator.getCurrentPose();

        double x = estimatedPose.getX();
        double y = estimatedPose.getY();
        Rotation2d angle = estimatedPose.getRotation();
       
        PhotonPipelineResult result = vision.getCamera().getLatestResult();
        int id;
        double tagX = 0;
        double tagY = 0; 

        if(result.hasTargets()){
            id = vision.getCamera().getLatestResult().getBestTarget().getFiducialId();


            Pose2d tagPose = vision.return_tag_pose(id).toPose2d();
            tagX = tagPose.getX();
            tagY = tagPose.getY();
        }
        else{
            id = -1;
        }

        double offset = Units.inchesToMeters(5);


        if(id == 6 || id == 7 || id == 8){
            System.out.println("SEEING BLUE ALLIANCE");
            
            return PathPlanner.generatePath(
                new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, 
                Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq),
                new PathPoint(new Translation2d(x, y), new Rotation2d(), angle),
                new PathPoint(new Translation2d(tagX + Units.inchesToMeters(32), tagY - offset + y_translation), new Rotation2d(), new Rotation2d(Units.degreesToRadians(180)))
            );
        }
        else if(id == 1 || id == 2 || id == 3){
            System.out.println("SEEING RED ALLIANCE");
            return PathPlanner.generatePath(
                new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, 
                Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq),
                new PathPoint(new Translation2d(x, y), new Rotation2d(), angle),
                new PathPoint(new Translation2d(tagX - Units.inchesToMeters(35), tagY + offset -  y_translation), new Rotation2d(), new Rotation2d(Units.degreesToRadians(0)))
            );
        }
        else {
            System.out.println("bReh");

            return PathPlanner.generatePath(
                new PathConstraints(Constants.DrivetrainConstants.kMaxVelocityMetersPerSecond, 
                Constants.DrivetrainConstants.maxAccelerationMetersPerSecondSq),
                new PathPoint(new Translation2d(x, y), new Rotation2d(), angle),
                new PathPoint(new Translation2d(x + 0.01, y), new Rotation2d(), angle)
            );
        }

    }
}