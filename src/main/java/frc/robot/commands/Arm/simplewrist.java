// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class simplewrist extends Command {
  /** Creates a new simplewrist. */
  
   private final Arm sys_wrist;
   private  double psyduck,pos; // psyduck percent power wrist 
   
  public simplewrist( Arm subsystem, double pwrw ) {
   
   sys_wrist=subsystem;
   psyduck=pwrw;
   
   addRequirements(sys_wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //sys_wrist.wristDC(psyduck);



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    pos = sys_wrist.get_wrist_position()*psyduck;
     sys_wrist.MoveWrist(pos+3);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //sys_wrist.wristDC(0);
  
  sys_wrist.MoveWrist(pos);
 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
