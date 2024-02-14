// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.k_CONTROLLERS;
import frc.robot.Constants.k_DRIVE;
import frc.robot.subsystems.Drive;

public class TeleopDrive extends Command {
  /** Creates a new TeleopDrive. */
  private final Drive sys_Drive;
  private DoubleSupplier pwr_f, pwr_r, turn;
  private double speed, angle, radius, dir, turn_db;

  public TeleopDrive(Drive subsystem, DoubleSupplier pwr_axis_f,DoubleSupplier pwr_axis_r, DoubleSupplier turn_axis) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_Drive = subsystem;
    pwr_f = pwr_axis_f;
    pwr_r = pwr_axis_r;
    turn = turn_axis;
    addRequirements(sys_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     speed = MathUtil.applyDeadband(pwr_f.getAsDouble() - pwr_r.getAsDouble(),k_CONTROLLERS.JOYSTICK_DEADBAND / 4);
     turn_db = MathUtil.applyDeadband(turn.getAsDouble(), k_CONTROLLERS.JOYSTICK_DEADBAND);
     
     // squaring the inputs 
      double speed_f = Math.signum(speed)*Math.pow(Math.abs(speed), 2);
      double turn_f = Math.signum(turn_db)*Math.pow(Math.abs(turn_db), 2);
    //  dir = Math.signum(turn_db);

    // if(dir == 0){
    // radius = k_DRIVE.NOM_RADIUS * (1/turn_db -(1 *dir));
    // }
    // else{
    //   radius = k_DRIVE.NOM_RADIUS * (1/turn_db -(1 *dir));
    // }

    // sys_Drive.RadiusDrive(speed, radius, dir);
    
    sys_Drive.holdendrive(speed_f, turn_f);
    
    


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
