// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.valueCansparkmax;

public class excelAndDecelLoop extends CommandBase {
  /** Creates a new excelAndDecelLoop. */
  private final valueCansparkmax VCSM;
  private double applePie= 0;
  private int on = 1;

  public excelAndDecelLoop(valueCansparkmax valueCansparkmax) {
    // Use addRequirements() here to declare subsystem dependencies.
    VCSM = valueCansparkmax; 
    addRequirements(valueCansparkmax);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  //applePie = speed x 100
  @Override
  public void execute() {
    if (on == 1){
      applePie = applePie + 1;
      if (applePie>99){
        on = 0;
      }
    }
    if (on == 0){
      applePie = applePie - 1;
      if (applePie<1){
        on = 1;
      }
    }
    VCSM.setvalue(applePie/100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
