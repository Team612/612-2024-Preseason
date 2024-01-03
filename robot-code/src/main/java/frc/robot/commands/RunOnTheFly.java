// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;

public class RunOnTheFly extends CommandBase {
  private final Limelight m_Limelight;
  private final PoseEstimator poseEstimatorSystem;
  private final Drivetrain m_swerve;
  private final boolean resetOdom;
  private final TrajectoryCreation m_traj;
  private final double translation;

  private CommandBase controllerCommand = Commands.none();

  /** Creates a new RunOnTheFly. */
  public RunOnTheFly(Drivetrain s, PoseEstimator p, boolean r, TrajectoryCreation traj, Limelight v, 
                    double y) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = s;
    poseEstimatorSystem = p;
    resetOdom = r;
    m_traj = traj;
    m_Limelight = v;
    translation = y;

    addRequirements(s, v, p);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerTrajectory path = m_traj.onthefly(m_swerve.getPoseEstimator(), m_Limelight, translation);

    if(resetOdom){
      m_swerve.resetPoseEstimator();
    }

    controllerCommand = Drivetrain.runTrajectory(m_swerve, path); //returns a trajectory based off the path
    controllerCommand.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}
