// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.SwerveLib.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(327.48046875);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(286.34765625);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(55.01953125);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(67.939453125);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
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
    public static final double LowPositionPivot = 38;
    public static final double MidPositionCubePivot = 95;
    public static final double HighPositionCubePivot = 141;
    public static final double MidPositionConePivot = 128;
    public static final double HighPositionConePivot = 140;
    public static final double HumanStationIntakePivot = 114;//123;
    public static final double GroundIntakePivot = 38; //tbd

    public static final double LowPositionTele = 88;
    public static final double MidPositionCubeTele = 45;
    public static final double HighPositionCubeTele = 37;
    public static final double MidPositionConeTele = 91;
    public static final double HighPositionConeTele = 110;
    public static final double HumanStationIntakeTele = 0;
    public static final double GroundIntakeTele = 88; //tbd


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
