// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class simplearmdrive extends Command {
  /** Creates a new simplearmdrive. */
  private  final Arm sys_arm;
  private  double pctpwr,pos;
 

  public simplearmdrive(Arm subsystem, double pwr){

  sys_arm=subsystem;
  pctpwr=pwr;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sys_arm);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 // sys_arm.shoulderDC(pctpwr);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   pos = sys_arm.get_arm_position()*pctpwr;
   sys_arm.Move2Angle(pos+3);  


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //sys_arm.shoulderDC(0);
  //double pos = sys_arm.get_arm_position();
  sys_arm.Move2Angle(pos);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
