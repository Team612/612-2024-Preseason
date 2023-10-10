// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private final ShuffleBoardButtons m_BoardButtons = new ShuffleBoardButtons();

  UsbCamera limelight;
  

  //Driverstation always sends out a red alliance which is weird so we wait until something is different
  public static Alliance initAllianceColor = Alliance.Invalid;

  private void checkDSUpdate() {
    Alliance currentAlliance = DriverStation.getAlliance();
    // If we have data, and have a new alliance from last time
    if (DriverStation.isDSAttached() && currentAlliance != Constants.DrivetrainConstants.alliance) {
      initAllianceColor = currentAlliance;
      
    }
  }

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_BoardButtons.initButtons();
    checkDSUpdate();
    m_robotContainer = new RobotContainer();

    // System.out.println("********ROBOT INIT*********");
    PathPlannerServer.startServer(5800);
    limelight = CameraServer.startAutomaticCapture();
    limelight.setFPS(80);
  



  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    checkDSUpdate();
    m_BoardButtons.updateButtons();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
    m_BoardButtons.updateButtons();
  }
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // System.out.println(    SmartDashboard.getBoolean("Mittens Toggle", false)    );
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    checkDSUpdate();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.TeleopHeading();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // m_robotContainer.TeleopHeading();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_BoardButtons.updateButtons();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
