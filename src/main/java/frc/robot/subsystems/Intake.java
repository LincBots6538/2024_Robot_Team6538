// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.k_INTAKE;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonSRX bag_Rol = new TalonSRX(k_INTAKE.MotorID_Rol);


  public Intake() {

    bag_Rol.configPeakCurrentLimit(0);
    bag_Rol.configContinuousCurrentLimit(k_INTAKE.CURRENT_LIMIT_NORMAL);
    bag_Rol.enableCurrentLimit(true);

    RollerStop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Roller(double pwr){
    bag_Rol.set(ControlMode.PercentOutput, pwr);
  }

  public void RollerStop(){
    bag_Rol.set(ControlMode.PercentOutput,0);
  }
}
