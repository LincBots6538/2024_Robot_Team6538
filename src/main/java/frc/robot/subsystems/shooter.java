// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.k_SHOOTER;

public class shooter extends SubsystemBase {
  /** Creates a new shooter. */
 private CANSparkMax neo_front=new CANSparkMax(k_SHOOTER.MotorID_BOT, MotorType.kBrushless);
 private CANSparkMax neo_back=new CANSparkMax(k_SHOOTER.MotorID_TOP, MotorType.kBrushless);
 private SparkPIDController frontPID,backPID;
 private RelativeEncoder encoder_front,encoder_back;
 

  public shooter() {

  neo_back.restoreFactoryDefaults();
  neo_front.restoreFactoryDefaults();
  frontPID=neo_front.getPIDController();
  backPID=neo_back.getPIDController();
  encoder_back=neo_back.getEncoder();
  encoder_front=neo_front.getEncoder();

  frontPID.setP(k_SHOOTER.kP);
  frontPID.setI(k_SHOOTER.kI);
  frontPID.setD(k_SHOOTER.kD);
  frontPID.setFF(k_SHOOTER.kF);
  backPID.setP(k_SHOOTER.kP);
  backPID.setI(k_SHOOTER.kI);
  backPID.setD(k_SHOOTER.kD);
  backPID.setFF(k_SHOOTER.kF);

  frontPID.setOutputRange(k_SHOOTER.min_output,k_SHOOTER.max_output);
  backPID.setOutputRange(k_SHOOTER.min_output, k_SHOOTER.max_output);



  }

 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void set_speed(double speed){

  frontPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
  backPID.setReference(speed, CANSparkMax.ControlType.kVelocity);



  }



}
