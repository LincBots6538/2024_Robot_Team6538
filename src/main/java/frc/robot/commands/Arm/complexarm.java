// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class complexarm extends Command {
  /** Creates a new complexarm. */

  private final  Arm sys_arm;
  private final  double degree;

  public complexarm(Arm subsystem, double angle) {

    sys_arm=subsystem;
    degree=angle;
  
  
   SmartDashboard.putNumber("angle", angle);
  
  
    // Use addRequirements() here to declare subsystem dependencies.
  
    addRequirements(sys_arm); 
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   sys_arm.Move2Angle(degree);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //sys_arm.Move2Angle(0);
  //sys_arm.Move2Angle(degree);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

   




}
