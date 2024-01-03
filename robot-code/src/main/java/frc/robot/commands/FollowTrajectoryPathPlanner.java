// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class FollowTrajectoryPathPlanner extends CommandBase {

  private Drivetrain s_swerve;
  private final PoseEstimator poseEstimatorSystem;
  private final String pathName;
  private final PathConstraints constraints;
  private final boolean resetOdom;
  private final boolean isBlueAlliance;
  private SwerveControllerCommand swerveController;
  private ProfiledPIDController thetaController;

  private CommandBase controllerCommand = Commands.none();

  /** Creates a new FollowTrajectoryPathPlanner. */
  public FollowTrajectoryPathPlanner(Drivetrain s, PoseEstimator p, String pathName, PathConstraints constraints, boolean resetOdom, boolean isBlueAlliance) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_swerve = s;
    this.poseEstimatorSystem = p;
    this.pathName = pathName;
    this.constraints = constraints;
    this.resetOdom = resetOdom;
    this.isBlueAlliance = isBlueAlliance;

    addRequirements(s_swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerTrajectory path = PathPlanner.loadPath(pathName, constraints);
    if(path == null){
      end(false);
      return;
    }

   

    Alliance alliance;

    if(isBlueAlliance){
      alliance = Alliance.Blue;
    }
    else{
      alliance = Alliance.Blue;
    }

    thetaController = new ProfiledPIDController(Constants.Swerve.kPTheta,
     Constants.Swerve.driveKI,
      Constants.Swerve.driveKD,
       new TrapezoidProfile.Constraints(Constants.Swerve.kMaxVelocityMetersPerSecond, Constants.Swerve.kMaxAccelerationMetersPerSecondSq)
       );

    PIDController xController = new PIDController(Constants.Swerve.driveKP,
    Constants.Swerve.driveKI,
    Constants.Swerve.driveKD); 
    PIDController yController = new PIDController(Constants.Swerve.driveKP, 
    Constants.Swerve.driveKI, 
    Constants.Swerve.driveKD); 
    
    PathPlannerTrajectory alliancePath = PathPlannerTrajectory.transformTrajectoryForAlliance(path, alliance);
    Trajectory traj = alliancePath;

 
   
    swerveController = new SwerveControllerCommand(traj, s_swerve.getRobotPosition() , Constants.Swerve.swerveKinematics, xController , yController, thetaController, null, s_swerve);
    controllerCommand = swerveController;
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
