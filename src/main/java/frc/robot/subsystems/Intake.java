// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.k_INTAKE;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonSRX bag_Rol = new TalonSRX(k_INTAKE.MotorID_Rol);
  private Timer curTime = new Timer();
  private double current=0, power =0;
  private boolean Jammed = false, on = false;

  public Intake() {

    bag_Rol.configPeakCurrentLimit(k_INTAKE.CURRENT_LIMIT_PEAK);
    bag_Rol.configPeakCurrentDuration(k_INTAKE.CURRENT_LIMIT_TIME);
    bag_Rol.configContinuousCurrentLimit(k_INTAKE.CURRENT_LIMIT_NORMAL);
    bag_Rol.enableCurrentLimit(true);

    RollerStop();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    current = bag_Rol.getSupplyCurrent();
    if (power > 0) {on = true;}

    SmartDashboard.putBoolean("Intake Rollers", on);
    SmartDashboard.putBoolean("Intake Jammed", Jammed);
    SmartDashboard.putNumber("Intake Current", current);
    if ((power > 0) &&(current < 10)){
      curTime.restart();
    }
    if ((current > 10) && (curTime.hasElapsed(0.700))){
      RollerStop();
      Jammed = true;
    }
    
  }

  public boolean ForcedStop(){
    if (Jammed){
      Jammed = false;
      return true;
    } else {
      return false;
    }
  }

  public void Roller(double pwr){
    power = Math.abs(pwr);
    if (power == 0) {
      on = false;
    } else {
      Jammed = false;
    }
    bag_Rol.set(ControlMode.PercentOutput, pwr);
  }

  public void RollerStop(){
    power = 0;
    on = false;
    bag_Rol.set(ControlMode.PercentOutput,0);
  }
}
