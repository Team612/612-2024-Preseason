// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Proxy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Drivetrain.TrajectoryCreation;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.Drivetrain.RunOnTheFly;
import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.Drivetrain.FieldOrientedDrive;
import frc.robot.commands.Drivetrain.FollowTrajectoryPathPlanner;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

import frc.robot.controls.ControlMap;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  //Drive subsystems declarations 
  private final Swerve m_swerve = Swerve.getInstance();
  //private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  private final DefaultDrive m_defaultdrive = new DefaultDrive(m_swerve);


 // Trajectories
  private final TrajectoryCreation m_traj = new TrajectoryCreation();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  //Subsystems
  
  private final FieldOrientedDrive m_FieldOrientedDrive = new FieldOrientedDrive(m_swerve);
  //public final Vision m_Vision = Vision.getVisionInstance();



 

 

  //public final Vision m_Vision = new Vision(camera);

  public final PoseEstimator estimator = PoseEstimator.getPoseEstimatorInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_gunnerController =
      new CommandXboxController(OperatorConstants.kGunnerControllerPort);

  


  // private final SequentialCommandGroup m_RedTopScoreAndLeave = new SequentialCommandGroup(
  //   boop
  //   .andThen(new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedTopLeave", Constants.DrivetrainConstants.constraint, true, false))
  // );

  // private final SequentialCommandGroup m_RedBottomScoreAndLeave = new SequentialCommandGroup(
  //   boop
  //   .andThen(new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedBottomLeave", Constants.DrivetrainConstants.constraint, true, false))
  // );

  // private final SequentialCommandGroup m_BlueTopScoreAndLeave = new SequentialCommandGroup(
  //   boop
  //   .andThen(new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueTopLeave", Constants.DrivetrainConstants.constraint, true, false))
  // );

  // private final SequentialCommandGroup m_BlueBottomScoreAndLeave = new SequentialCommandGroup(
  //   boop
  //   .andThen(new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueBottomLeave", Constants.DrivetrainConstants.constraint, true, false))
  // );

  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureShuffleBoardBindings();
    configureDefaultCommands();
  }


  private void configureShuffleBoardBindings(){
    // m_chooser.addOption("Auto-Balance", new DockingSequence(m_drivetrain));
    // m_chooser.addOption("Red Top Leave And Dock", new ProxyCommand(() -> m_RedTopLeaveAndDock));
    // m_chooser.addOption("Blue Top Leave And Dock", new ProxyCommand(() -> m_BlueTopLeaveAndDock));
    // m_chooser.addOption("Red Bottom Leave And Dock", new ProxyCommand(() -> m_RedBottomLeaveAndDock));
    // m_chooser.addOption("Blue Bottom Leave and Dock", new ProxyCommand(() -> m_BlueBottomLeaveAndDock));

  
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putBoolean("Mittens Toggle", true);
  }

  private void configureButtonBindings() {

    m_driverController.back().toggleOnTrue(m_defaultdrive);

   

   

    // ControlMap.green1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, Units.inchesToMeters(-34))));
    


  }


  private void configureDefaultCommands(){
    //m_drivetrain.setDefaultCommand(m_FieldOrientedDrive);
    m_swerve.setDefaultCommand(m_FieldOrientedDrive);
  

  }

  public void TeleopHeading(){
  }
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
