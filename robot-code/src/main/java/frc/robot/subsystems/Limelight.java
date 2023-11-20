// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
public class Limelight extends SubsystemBase {
  private NetworkTable limelightTable;
  private static Limelight instance;
  /** Creates a new Limelight. */
  public Limelight() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-sixtwel");
  }
  

  public void switchPipline(int n){ // 0 <= n <= 9
    limelightTable.getEntry("pipeline").setNumber(n);
  }


  //getters
  public double getTx(){ //horizontal deviation from the center. -27 - 27 degrees
    return limelightTable.getEntry("tx").getDouble(0.0);
  }

  public double getTy(){ //vertical deviation from the center. -27 - 27 degrees
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  public double getTa(){ //target area, 0% of the screen - 100% of the screen
    return limelightTable.getEntry("ta").getDouble(0.0);

  }

  public boolean getTv(){ //valid targets true or false
    if (limelightTable.getEntry("tv").getDouble(0) == 1){
      return true;
    }
    return false;

  }



  public static Limelight getInstance(){
    if (instance == null){
      instance = new Limelight();
    }
    return instance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}