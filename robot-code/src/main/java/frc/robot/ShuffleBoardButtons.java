// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
public class ShuffleBoardButtons {
    ShuffleboardTab m_driverTab;
    ShuffleboardTab m_encoderTab;
    ShuffleboardTab m_graphTab;
    ShuffleboardTab m_debugTab;
    ShuffleboardTab m_limitSwitchTab;
    
    GenericEntry NavxAngle;
    GenericEntry PoseEstimatorAngle;
    GenericEntry PoseEstimatorX;
    GenericEntry PoseEstimatorY;
    GenericEntry fodState;
    // GenericEntry grabberCurrentGraph;
    GenericEntry telescopeCurrentGraph;
    GenericEntry BoreEncoders;
    GenericEntry pivotEntry;
    GenericEntry telescopeEntry;
    GenericEntry telescopeEncoderRate;
    GenericEntry isGrabbing;
    GenericEntry isReleasing;

    GenericEntry pivotBLS;
    GenericEntry telescopeLimitSwitch;

    GenericEntry isBlueAlliance;

    public static GenericEntry toggleAlliance;
    

    //accessable entires
    public static GenericEntry grabberSpikeTresh;
    public static GenericEntry teleEncoderRateThresh;

    public static GenericEntry teleSpikeThresh;

    //debugging for arm thresholds
    public static GenericEntry lowGeneral;
    public static GenericEntry midCube;
    public static GenericEntry highCube;
    public static GenericEntry midCone;
    public static GenericEntry highCone;
    public static GenericEntry humanStation; //telescope debug 
    public static GenericEntry ground;

    

    public void initButtons(){
        m_driverTab = Shuffleboard.getTab("DriverTab");
        m_encoderTab = Shuffleboard.getTab("Encoder");
        m_graphTab = Shuffleboard.getTab("Graphs");
        m_debugTab = Shuffleboard.getTab("Debug Tab");
        m_limitSwitchTab = Shuffleboard.getTab("Limit Switch Tab");
        

        //debug entries

        NavxAngle = m_debugTab.add("NavX angle", 0.0).getEntry();
        PoseEstimatorAngle = m_debugTab.add("PoseEstimator Angle", 0.0).getEntry();
        PoseEstimatorX = m_debugTab.add("PoseEstimator X", 0.0).getEntry();
        PoseEstimatorY = m_debugTab.add("PoseEstimator Y", 0.0).getEntry();
        isBlueAlliance = m_debugTab.add("isBlueAlliance", false).getEntry();


        lowGeneral = m_debugTab.add("low General", 0.0).getEntry();
        midCube = m_debugTab.add("midCube", 0.0).getEntry();
        highCube = m_debugTab.add("highCube", 0.0).getEntry();
        midCone = m_debugTab.add("mideCone", 0.0).getEntry();
        highCone = m_debugTab.add("highCone", 0.0).getEntry();
        humanStation = m_debugTab.add("humanStation", 0.0).getEntry();
        ground = m_debugTab.add("ground", 0.0).getEntry();

        //smartdashboard entries
        
        fodState = m_driverTab.add("FOD state?", false).getEntry();
        isGrabbing = m_driverTab.add("isGrabbing", false).getEntry();
        isReleasing = m_driverTab.add("isReleasing", false).getEntry();


        // encoder entries
        pivotEntry = m_encoderTab.add("Pivot Encoder", 0.0).getEntry();
        BoreEncoders = m_encoderTab.add("Bore Encoder",0.0).getEntry();

        //graphing entries
        grabberSpikeTresh = m_graphTab.add("GrabberSpikeTresh",0.0).getEntry();
        // grabberCurrentGraph = m_graphTab.add("Grabber Current vs Time", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        telescopeCurrentGraph = m_graphTab.add("Telescope Current vs Time", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        telescopeEncoderRate = m_graphTab.add("Telescope Encoder Rate", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();
        telescopeEntry = m_encoderTab.add("Telescope Encoder", 0.0).getEntry();
        teleEncoderRateThresh = m_graphTab.add("TeleEncoderThresh", 0.0).getEntry();
        teleSpikeThresh = m_graphTab.add("TeleSpikeThresh", 0.0).getEntry();

        pivotBLS = m_limitSwitchTab.add("pivotLimitSwitchState", false).getEntry();

        telescopeLimitSwitch = m_limitSwitchTab.add("telescopeLimitSwitch", false).getEntry();
        toggleAlliance = m_driverTab.add("Is Red Alliance?",false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }

    public void updateButtons(){
        Drivetrain drivetrain = Drivetrain.getInstance();
        PoseEstimator poseEstimator = PoseEstimator.getPoseEstimatorInstance();
        Pose2d estimator = poseEstimator.getCurrentPose();
        
        NavxAngle.setDouble(drivetrain.getNavxAngle().getDegrees());
        PoseEstimatorAngle.setDouble(estimator.getRotation().getDegrees());
        PoseEstimatorX.setDouble(estimator.getX());
        PoseEstimatorY.setDouble(estimator.getY());
        fodState.setBoolean(drivetrain.getFodState());
    }
    
}