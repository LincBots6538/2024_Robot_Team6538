// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.k_SETPOINTS;
import frc.robot.commands.complexarm;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmPosistions extends SequentialCommandGroup {
  /** Creates a new SetArmPosistions. */
  public SetArmPosistions(Arm subsytem, int position) {
    
    double benderarm_pos = k_SETPOINTS.ARM_HOME;
    double benderwrist_pos = k_SETPOINTS.WRIST_HOME;
    double wait = 0;
    
    if (subsytem.get_wrist_position()<0){
     wait=1.0;

    }

    switch (position){
    
      case 0:     // home position
        benderarm_pos=k_SETPOINTS.ARM_HOME;
        benderwrist_pos=k_SETPOINTS.WRIST_HOME;
          break;
      case 1:    // intake 
        benderarm_pos=k_SETPOINTS.ARM_INTAKE;
        benderwrist_pos=k_SETPOINTS.WRIST_INTAKE;
          break;
      case 2:     // speaker
        benderarm_pos=k_SETPOINTS.ARM_SPEAKER;
        benderwrist_pos=k_SETPOINTS.WRIST_SPEAKER;
          break;
      case 3:     // amp
        benderarm_pos=k_SETPOINTS.ARM_AMP;
        benderwrist_pos=k_SETPOINTS.WRIST_AMP;
          break;
      case 4:   // climb 
        benderarm_pos=k_SETPOINTS.ARM_CLIMB;
        benderwrist_pos=k_SETPOINTS.WRIST_CLIMB;
          break;
      default:
        benderarm_pos = subsytem.get_arm_position();
        benderwrist_pos = subsytem.get_wrist_position();
    }
    
    
    
    if (benderwrist_pos<0) {
      wait = 1;
      addCommands(new complexarm(subsytem, benderarm_pos),
        new WaitCommand(wait),
        new complexwrist(subsytem, benderwrist_pos));
      
    } else{
        addCommands(new complexwrist(subsytem, benderwrist_pos),
         new WaitCommand(wait),
         new complexarm(subsytem, benderarm_pos));
    }
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  }
}
