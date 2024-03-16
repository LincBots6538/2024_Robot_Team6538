// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.k_SHOOTER;

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */
 private CANSparkMax neo_front=new CANSparkMax(k_SHOOTER.MotorID_BOT, MotorType.kBrushless);
 private CANSparkMax neo_back=new CANSparkMax(k_SHOOTER.MotorID_TOP, MotorType.kBrushless);
 private SparkPIDController frontPID,backPID;
 private RelativeEncoder encoder_front,encoder_back;
 private double top_speed, bot_speed, tgt_speed=0;
 private boolean Locked = false;
 

  public shooter() {
    // Reset Spark Max Settings
    neo_back.restoreFactoryDefaults();
    neo_front.restoreFactoryDefaults();

    // Flip Positive Direction
    neo_front.setInverted(true);
    neo_back.setInverted(true);

    // Setup PID Controllers
    frontPID=neo_front.getPIDController();
    backPID=neo_back.getPIDController();

    // Velocity Control
    frontPID.setP(k_SHOOTER.kP,0);
    frontPID.setI(k_SHOOTER.kI,0);
    frontPID.setD(k_SHOOTER.kD,0);
    frontPID.setFF(k_SHOOTER.kF,0);
    frontPID.setOutputRange(k_SHOOTER.min_output,k_SHOOTER.max_output,0);

    backPID.setP(k_SHOOTER.kP,0);
    backPID.setI(k_SHOOTER.kI,0);
    backPID.setD(k_SHOOTER.kD,0);
    backPID.setFF(k_SHOOTER.kF,0);
    backPID.setOutputRange(k_SHOOTER.min_output, k_SHOOTER.max_output,0);
    
    // Position Control
    frontPID.setP(k_SHOOTER.kP_POS,1);
    frontPID.setI(0,1);
    frontPID.setD(0,1);
    frontPID.setFF(0,1);
    frontPID.setOutputRange(k_SHOOTER.min_output,k_SHOOTER.max_output,1);

    backPID.setP(k_SHOOTER.kP_POS,1);
    backPID.setI(0,1);
    backPID.setD(0,1);
    backPID.setFF(0,1);
    backPID.setOutputRange(k_SHOOTER.min_output, k_SHOOTER.max_output,1);

    
    // Declare Encoder Objects
    encoder_back=neo_back.getEncoder();
    encoder_front=neo_front.getEncoder();


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    top_speed = encoder_back.getVelocity();
    bot_speed = encoder_front.getVelocity();
    SmartDashboard.putNumber("Top Wheel Speed", top_speed / k_SHOOTER.GEAR_RATIO);
    SmartDashboard.putNumber("Bottom Wheel Speed", bot_speed / k_SHOOTER.GEAR_RATIO);
    SmartDashboard.putNumber("Target Wheel Speed", tgt_speed / k_SHOOTER.GEAR_RATIO);

    if ((tgt_speed == 0) && (!Locked)) {    // Is it commanded to zero
      if((Math.abs(top_speed) < 50)&&(Math.abs(bot_speed) < 50)){   //has it stopped
        wheelLock();
      } 
    }
  }


  public void set_speed(double speed){

    frontPID.setReference(speed, ControlType.kVelocity,0,0);
    backPID.setReference(speed, ControlType.kVelocity,0,0);
    tgt_speed = speed;
    Locked = false;
  }

  public boolean OnTarget(){
    //boolean top = (Math.abs(top_speed - tgt_speed) < k_SHOOTER.SPEED_ERR);
    //boolean bot = (Math.abs(bot_speed - tgt_speed) < k_SHOOTER.SPEED_ERR);
    boolean top = (top_speed > tgt_speed);
    boolean bot = (bot_speed > tgt_speed);
    return (top || bot);
  }

  /** Sets the Shooter Wheels to stay at a position */
  public void wheelLock(){
    double top_pos = encoder_back.getPosition();
    double bot_pos = encoder_front.getPosition();

    frontPID.setReference(bot_pos, ControlType.kPosition, 1,0);
    backPID.setReference(top_pos, ControlType.kPosition, 1,0);

    Locked = true;
  }
}
