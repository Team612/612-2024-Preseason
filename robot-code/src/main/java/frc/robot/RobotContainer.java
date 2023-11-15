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
import frc.robot.LedCommands.LedMed;
import frc.robot.LedCommands.MakeTheRainbow;
import frc.robot.LedCommands.Purple;
// import main.java.frc.robot.subsystems.intake;
import frc.robot.LedCommands.TeleopDefault;
import frc.robot.LedCommands.Yellow;
import frc.robot.commands.Drivetrain.AutoBalance;
import frc.robot.commands.Drivetrain.AutoDriveThingy;
import frc.robot.commands.Drivetrain.AutoTurnThingy;
// import AutoDriveThingy;
import frc.robot.commands.Drivetrain.Boop;
import frc.robot.commands.Drivetrain.DefaultDrive;
import frc.robot.commands.Drivetrain.DockingSequence;
import frc.robot.commands.Drivetrain.FollowTrajectory;
import frc.robot.commands.Drivetrain.SetForward;
import frc.robot.commands.Drivetrain.SlowmoDrive;
import frc.robot.commands.Drivetrain.TrajectoryCreation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.led;
// import main.java.frc.robot.commands.Drivetrain.SetSpeed;
// import main.java.frc.robot.commands.Drivetrain.SetSpeed2;
// import main.java.frc.robot.subsystems.intake;
// import main.java.frc.robot.subsystems.motorthingy;
import frc.robot.commands.Drivetrain.RollOff;
import frc.robot.commands.Drivetrain.RunOnTheFly;
import frc.robot.commands.Grab;
import frc.robot.commands.Pivot;
import frc.robot.commands.Release;
import frc.robot.commands.ExtendRetract;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Telescope;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.Drivetrain.FieldOrientedDrive;
import frc.robot.commands.Drivetrain.FollowTrajectoryPathPlanner;
import frc.robot.commands.PivotPositions.DefenseMode;
import frc.robot.commands.PivotPositions.ExtendToPosition;
import frc.robot.commands.ReleaseAuto;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.commands.PivotPositions.MoveToPosition;
import frc.robot.controls.ControlMap;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  //Drive subsystems declarations 
  private final Drivetrain m_drivetrain = Drivetrain.getInstance();
  // private final motorthingy m_motorthingy = new motorthingy();
  // private final motorthingy m_intake = new intake();
  private final DefaultDrive m_defaultdrive = new DefaultDrive(m_drivetrain);
  private final FieldOrientedDrive m_FieldOrientedDrive = new FieldOrientedDrive(m_drivetrain);

 // Trajectories
  private final FollowTrajectory m_follower = new FollowTrajectory();
  private final TrajectoryCreation m_traj = new TrajectoryCreation();
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  //Subsystems
  private final Arm m_arm = Arm.getInstance();
  private final Telescope m_scope = Telescope.getInstance();
  private final Grabber m_grabber = Grabber.getInstance();
  private final led m_Led = led.getLEDInstance();
  public final Vision m_Vision = Vision.getVisionInstance();


  //Commands
  private final Pivot m_pivot = new Pivot(m_arm);
  private final ExtendRetract m_telescope = new ExtendRetract(m_scope, m_arm);
  private final Grab m_grab = new Grab(m_grabber);
  private final Release m_release = new Release(m_grabber);
  private final ReleaseAuto m_releaseauto = new ReleaseAuto(m_grabber);
  private final AutoBalance m_autoBalance = new AutoBalance(m_drivetrain);
  
  private final TeleopDefault m_TeleopDefault = new TeleopDefault(m_Led, m_Vision);

  //gunner outtakes/defense mode
  private final Command m_midCone = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.MidPositionConePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.MidPositionConeTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());
  
  private final Command m_midCube = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.MidPositionCubePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.MidPositionCubeTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());

  private final Command m_highCube = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.HighPositionCubePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.HighPositionCubeTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());
    
  private final Command m_humanStation = new SequentialCommandGroup(
    new MoveToPosition(m_arm, 0.7, EncoderConstants.HumanStationIntakePivot).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.HumanStationIntakeTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());


    
  private final Command testLed = new LedMed(m_Led, 100);
  private final Command testled2 = new MakeTheRainbow(m_Led, m_drivetrain);
  // private final Command DoubleSolenoide = new doublesolenoid(m_intake);
  // private final Command testLede = new LedMed(m_Led, 100);
  // private final Command testLede = new;
    // private final SequentialCommandGroup m_highCone = new SequentialCommandGroup(
    //   new ExtendToPosition(m_scope, 0.7, 0).
    //   andThen(new MoveToPosition(m_arm, 0.7, EncoderConstants.HighPositionCubePivot)).
    //   andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.HighPositionConeTele)));

  private final Command m_lowGeneral = new SequentialCommandGroup(
    new ExtendToPosition(m_scope, 0.7, 0).
    andThen(new MoveToPosition(m_arm, 0.7, EncoderConstants.LowPositionPivot)).
    andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.LowPositionTele))).
    until(() -> Math.abs(ControlMap.gunner_joystick.getRawAxis(1)) >= 0.1 || ControlMap.GUNNER_RB.getAsBoolean() || ControlMap.GUNNER_LB.getAsBoolean());
    
  private final DefenseMode m_DefenseMode = new DefenseMode(m_scope, 0.7); //write override here
  
  private final ParallelCommandGroup m_stow = new ParallelCommandGroup(new DefenseMode(m_scope, 0.5).alongWith(new MoveToPosition(m_arm, 0.3, 0)));
 
  private final Command m_boop = new SequentialCommandGroup(
    new DefenseMode(m_scope,0.5)
    .andThen(new MoveToPosition(m_arm, 0.6, 20))
    .andThen(new DefenseMode(m_scope,0.5))
    .andThen(new MoveToPosition(m_arm, 0.6, 0))
  );

  private final Command m_autoScore = new SequentialCommandGroup(new DefenseMode(m_scope, 0.1)
    .andThen(new Grab(m_grabber)))
    .andThen(
      new ParallelCommandGroup(new SequentialCommandGroup(
        new MoveToPosition(m_arm, 0.7, EncoderConstants.MidPositionConePivot)
          .andThen(new ExtendToPosition(m_scope, 0.7, EncoderConstants.MidPositionConeTele)))
      .alongWith(new Grab(m_grabber))))
  .andThen(new MoveToPosition(m_arm, 0.3, 100))
  .andThen(m_releaseauto)
  .andThen(m_stow);

  //public final Vision m_Vision = new Vision(camera);

  public final PoseEstimator estimator = PoseEstimator.getPoseEstimatorInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_gunnerController =
      new CommandXboxController(OperatorConstants.kGunnerControllerPort);

  
  private final SequentialCommandGroup m_RedMiddleLeaveAndDock = new SequentialCommandGroup(
    // boop.andthen(
    new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedMiddleLeaveAndDock", Constants.DrivetrainConstants.constraint, true, false)
    .andThen(new DockingSequence(m_drivetrain))
  );

  private final SequentialCommandGroup m_BlueMiddleLeaveAndDock = new SequentialCommandGroup(
    // boop.andthen(
    new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueMiddleLeaveAndDock", Constants.DrivetrainConstants.constraint, true, false)
    .andThen(new DockingSequence(m_drivetrain))
  );

  private final SequentialCommandGroup m_RomiRoutine = new SequentialCommandGroup(
     new AutoDriveThingy(m_drivetrain)
     .andThen(new AutoTurnThingy(m_drivetrain))
     .andThen(new AutoDriveThingy(m_drivetrain))
     .andThen(new AutoTurnThingy(m_drivetrain))
  );

  private final SequentialCommandGroup TheRobotIsHavingASeizure = new SequentialCommandGroup(
    // new AutoTurnThingy(m_drivetrain)
    // .andThen(new AutoTurnThingy(m_drivetrain))
    // .andThen(new AutoTurnThingy(m_drivetrain))
    // .andThen(new AutoLEDThingy(m_drivetrain))
 );
  
  private final SequentialCommandGroup MotorSpinnyWinny = new SequentialCommandGroup(
    //  new SetSpeed(m_drivetrain, m_motorthingy).andThen(new setSpeed2(m_drivetrain, m_motorthingy))
  );
  // // private final SequentialCommandGroup m_RedTopScoreAndLeave = new SequentialCommandGroup(
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
    m_chooser.addOption("Red Bottom Leave", new SequentialCommandGroup(new Boop(m_scope, m_arm).andThen(new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedBottomLeave", Constants.DrivetrainConstants.constraint, true, false)))));
    m_chooser.addOption("Red Top Leave", new SequentialCommandGroup(new Boop(m_scope, m_arm).andThen(new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "RedTopLeave", Constants.DrivetrainConstants.constraint, true, false)))));
   
    m_chooser.addOption("Blue Bottom Leave", new SequentialCommandGroup(new Boop(m_scope, m_arm).andThen(new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueBottomLeave", Constants.DrivetrainConstants.constraint, true, true)))));
    
    m_chooser.addOption("Blue Top Leave", new SequentialCommandGroup(new Boop(m_scope, m_arm).andThen(new ProxyCommand(() -> new FollowTrajectoryPathPlanner(m_drivetrain, estimator, "BlueTopLeave", Constants.DrivetrainConstants.constraint, true, true)))));
   
    m_chooser.addOption("Red Middle Leave and Dock", new SequentialCommandGroup(new Boop(m_scope, m_arm).andThen(new ProxyCommand(() -> m_RedMiddleLeaveAndDock))));
    m_chooser.addOption("Blue Middle Leave and Dock", new SequentialCommandGroup(new Boop(m_scope, m_arm).andThen(new ProxyCommand(() -> m_BlueMiddleLeaveAndDock))));
    
    m_chooser.addOption("auto score cone", m_autoScore);
    m_chooser.addOption("Ambulance Lights", testLed);
    m_chooser.addOption("rainbow", testled2);
    m_chooser.addOption("Srikar's test", m_RomiRoutine);
    // m_chooser.addOption("double solenoid", DoubleSolenoide);
    // m_chooser.addOption("Red Top Leave And Dock", new ProxyCommand(() -> m_RedTopLeaveAndDock));
    // m_chooser.addOption("Blue Top Leave And Dock", new ProxyCommand(() -> m_BlueTopLeaveAndDock));
    // m_chooser.addOption("Red Bottom Leave And Dock", new ProxyCommand(() -> m_RedBottomLeaveAndDock));
    // m_chooser.addOption("Blue Bottom Leave and Dock", new ProxyCommand(() -> m_BlueBottomLeaveAndDock));

  
    SmartDashboard.putData(m_chooser);
    // SmartDashboard.putData("Slowmo (Toggle)", new SlowmoDrive(m_drivetrain));
  }

  private void configureButtonBindings() {

    m_gunnerController.leftTrigger().whileTrue(m_grab);
    m_driverController.y().whileTrue(new SetForward(m_drivetrain));
    m_driverController.back().toggleOnTrue(m_defaultdrive);
    m_gunnerController.y().whileTrue(new Yellow(m_Led));
    m_gunnerController.x().whileTrue(new Purple(m_Led));
    ControlMap.red2.toggleOnTrue(new ProxyCommand(() -> m_autoBalance));

    ControlMap.blue1.toggleOnTrue(new ProxyCommand(() -> m_midCube));
    ControlMap.blue2.toggleOnTrue(new ProxyCommand(() -> m_highCube));
    ControlMap.red4.toggleOnTrue(new ProxyCommand(() -> m_midCone));
    ControlMap.red5.toggleOnTrue(new ProxyCommand(() -> m_humanStation));
    ControlMap.green2.toggleOnTrue(new ProxyCommand(() -> m_lowGeneral));

    ControlMap.green1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, Units.inchesToMeters(28.5)))); //robot oriented right cone
    ControlMap.yellow2.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, 0))); //april tag alignment
    ControlMap.yellow1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, Units.inchesToMeters(-22.5 -6)))); 

    // ControlMap.green1.toggleOnTrue(new ProxyCommand(() -> new RunOnTheFly(m_drivetrain, estimator, true, m_traj, m_Vision, Units.inchesToMeters(-34))));
    


    ControlMap.red6.toggleOnTrue(new ProxyCommand(() -> m_DefenseMode));
  }


  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(m_FieldOrientedDrive);
    m_arm.setDefaultCommand(m_pivot);
    m_scope.setDefaultCommand(m_telescope);
    m_grabber.setDefaultCommand(m_release);
    m_Led.setDefaultCommand(m_TeleopDefault.ignoringDisable(true));

  }

  public void TeleopHeading(){
    Rotation2d finalHeading = new Rotation2d(Units.degreesToRadians(-180));
    Rotation2d currentHeading = estimator.getCurrentPose().getRotation();
    Rotation2d deltaHeading = finalHeading.minus(currentHeading);
    if(Robot.initAllianceColor == Alliance.Blue){
    m_drivetrain.setNavxAngleOffset(deltaHeading.plus(new Rotation2d(Units.degreesToRadians(180))));}
    
    if(Robot.initAllianceColor == Alliance.Red){
      m_drivetrain.setNavxAngleOffset(deltaHeading.plus(new Rotation2d(Units.degreesToRadians(0))));}
  }
  
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
