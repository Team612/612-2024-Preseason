// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DrivetrainConstants{

    public static Alliance alliance = Alliance.Invalid;


     //Spark constants
     public final static int SPARK_FL = 2;
     public final static int SPARK_FR = 1;
     public final static int SPARK_BL = 4;
     public final static int SPARK_BR = 3;
     //TEMP VALUES
     public final static int SPARK_ANGLE_FL = 5;
     public final static int SPARK_ANGLE_FR = 6;
     public final static int SPARK_ANGLE_BL = 7;
     public final static int SPARK_ANGLE_BR = 8;
     //ENCODER VALUES
     public final static int ENCODER_ANGLE_FL = 0;
     public final static int ENCODER_ANGLE_FR = 1;
     public final static int ENCODER_ANGLE_BL = 2;
     public final static int ENCODER_ANGLE_BR = 3;

    //multipler for slowmo
    public static double slowmo = 1;
     //wheel diameter
     public static final double kWheelDiameterMeters = 0.1524; 
     
     //Distance between centers of right and left wheels on robot
     public static final double kTrackWidth = Units.inchesToMeters(26.625);//0.5969; // for phoenix 0.668655
 
     //Distance between centers of front and back wheels on robot
     public static final double kWheelBase =  Units.inchesToMeters(21);//  0.676275; // for phoenix .5334
 
 
     public static final double kEncoderCPR = 1; 
     public static final double kGearReduction = 16;
     
     //Finding Distance per pulse
     public static final double kEncoderDistancePerPulse =
         ((kWheelDiameterMeters * Math.PI)) / (kGearReduction);
  
     //Feedforward gains for system dynamics 
     public static final double kS = 0.11887; 
     public static final double kV = 4.1902;  
     public static final double kA = 0.38145;
    

     //Angular gains
     public static final double kV_Angular = 4.218; // do not touch
     public static final double kA_Angular = 0.15213; // do not touch
    //thresholds
    public static final int offbalancepositive = 15;
    public static final int offbalancepositivehalf = 5; //might need to change later
    public static final int offbalancepositivehalfneg = -5;
     
     //position controllers

     //have to tune manually
     public static final double kPXController = 5; // ~ 1cm error
     public static final double kPYController = 1;
     public static final double kPThetaController = 4.5; 
 
     
    //Velocity controllers
     public static final double kPFrontLeftVel = 4.875; 
     public static final double kPRearLeftVel = 4.875;
     public static final double kPFrontRightVel = 4.875;
     public static final double kPRearRightVel = 4.875;

     

 
     //Converting chassis velocity into individual wheel velocities
     public static final MecanumDriveKinematics kDriveKinematics =
         new MecanumDriveKinematics(
             new Translation2d(kWheelBase / 2, kTrackWidth / 2),
             new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
             new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
             new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
     );

     
     //trajectory constraints
     public static final double kMaxVelocityMetersPerSecond = 1;
     public static final double maxAccelerationMetersPerSecondSq = 1;

     public static final double kMaxAngularVelocity = Math.PI;
     public static final double kMaxAngularAcceleration = Math.PI;
 
     //angular constraints
     public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
         new TrapezoidProfile.Constraints(kMaxAngularVelocity, kMaxAngularAcceleration);
 
     //Feedforward 
     public static final SimpleMotorFeedforward kFeedforward =
         new SimpleMotorFeedforward(Constants.DrivetrainConstants.kS, Constants.DrivetrainConstants.kV, Constants.DrivetrainConstants.kA);
     
     //path planner
     public static final Alliance blueAlliance = Alliance.Blue;
     public static final Alliance redAlliance = Alliance.Red;
     
     public static final PathConstraints constraint = new PathConstraints(kMaxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);

  }

  public static class SwerveConstants {
    public static final double SwerveMaxSpeed = 5.0;//5 meters a second
    public final double[] fl_distance = {0.0,0.0};
    public final double[] fr_distance = {0.0,0.0};
    public final double[] bl_distance = {0.0,0.0};
    public final double[] br_distance = {0.0,0.0};
    public static final double angleConversionFactor = (8.14/1.0) / 360; //degree per pulse
    public static final double distancePerPulse = 8.14/ ((Math.PI * 4)); //distance per pulse
    //pid.. use pid tuner
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    //gains... need to be calculated manually
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;


  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static int kGunnerControllerPort = 1;

  }

  public static class SparkPorts {
     public static final int pivotID = 6;
     public static final int tele_arm = 7;
     public static final int grabber = 5;
   }
 
   public static class MotorSpeeds {
     public static final double pivot_speed = 1;
     public static final double tele_arm_speed = 0.5;
     public static double grabber_speed = 0.8;
    public static double slow_down_release = 0.6;
   }
 
   public static class EncoderConstants{
    public static final double MittensLowPositionPivot = 50;  //43
    public static final double RhinoLowPositionPivot = 43;  //43


    public static final double MidPositionCubePivot = 95; 
    public static final double HighPositionCubePivot = 141; //124
    public static final double MidPositionConePivot = 128;
    public static final double HighPositionConePivot = 140;
    public static final double HumanStationIntakePivot = 120;//114;

    public static final double LowPositionTele = 116; //87
    public static final double MidPositionCubeTele = 45;
    public static final double HighPositionCubeTele = 44; //37
    public static final double MidPositionConeTele = 91;
    public static final double HighPositionConeTele = 110;
    public static final double HumanStationIntakeTele = 0;


    public static int boreEncoderIntake = 1;
 
     //all of these are subject to change
     public static double arm_lower = -91.0; // = -99.0;
     public static double arm_upper = 100.0; // = 130.0;
     public static double arm_score_low = 89;
     public static double arm_score_high = 91;
     public static double tele_in; // = -3.0;
     public static double tele_out = 110; // = 84.0;
    public static double tele_extension_rate = 5.0;
    public static double tele_extension_rate_thresh = 1.0;
    public static double tele_motor_current = 10;
    public static double grabber_extension_rate = 1000.0;
    public static double grabber_extension_rate_thresh = 200.0;
    public static double grabber_motor_current = 10.0;

    public static double sticky_grabber_thresh = 10;

    public static double boop_thresh = 20;
  
   }

  public static class VisionConstants{
    public static String cameraName = "Microsoft_LifeCam_HD-3000";

    //constraints
    public static final TrapezoidProfile.Constraints ThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final TrapezoidProfile.Constraints PControllerConstraints =
        new TrapezoidProfile.Constraints(1.5,1.5);
    public static final TrapezoidProfile.Constraints StrafeControllerConstaints = 
        new TrapezoidProfile.Constraints(1.5, 1.5);
    
    //controllers
    public static final ProfiledPIDController rotationController = 
      new ProfiledPIDController(.01, 0, 0, ThetaControllerConstraints);
    public static final ProfiledPIDController forwardController = 
      new ProfiledPIDController(.5, 0, 0, PControllerConstraints);
    public static final ProfiledPIDController strafeController = 
      new ProfiledPIDController(.4, 0, 0, StrafeControllerConstaints);

    //other camera constants
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(13.5);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(14.25);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(12);
    public static final double GOAL_RANGE_METERS = 1;
    
    public static final Transform3d CAMERA_TO_Robot = new Transform3d();
  }
}
