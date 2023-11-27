package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private SwerveModule[] mSwerveMods;

//   private static AHRS navx;
//   private Rotation2d navxAngleOffset;

  private Field2d field;

  public Drivetrain() {
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    
//     navx = new AHRS(I2C.Port.kMXP); 
//     navxAngleOffset = new Rotation2d();
//     navx.reset();
//     navx.calibrate();

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
//     SwerveModuleState[] swerveModuleStates =
//         Constants.Swerve.swerveKinematics.toSwerveModuleStates(
//             fieldRelative
//                 ? ChassisSpeeds.fromFieldRelativeSpeeds(
//                     translation.getX(), translation.getY(), rotation, getYaw())
//                 : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

//   public void zeroGyro() {
//     navx.zeroYaw();
//   }

//   public Rotation2d getNavxAngle(){
//     return Rotation2d.fromDegrees(-navx.getAngle());
//   }
    
//   // setter for setting the navxAngleOffset
//   public void setNavxAngleOffset(Rotation2d angle){
//     navxAngleOffset = angle;
//   }

//   public Rotation2d getYaw(){
//     return Rotation2d.fromDegrees(navx.getYaw());
//   }
//   public Rotation2d getPitch(){
//     return Rotation2d.fromDegrees(navx.getPitch());
//   }

  @Override
  public void periodic() {
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}