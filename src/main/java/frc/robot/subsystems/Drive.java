// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.k_DRIVE;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  // Declare Varibles here i.e. Speed controllers
  private TalonFX f500_fr_drive = new TalonFX(k_DRIVE.MotorID_FR);
  private TalonFX f500_rr_drive = new TalonFX(k_DRIVE.MotorID_RR);
  private TalonFX f500_fl_drive = new TalonFX(k_DRIVE.MotorID_FL);
  private TalonFX f500_rl_drive = new TalonFX(k_DRIVE.MotorID_RL);

  // Velocity Control Setttings
  private VelocityVoltage l_velocityControl = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  private VelocityVoltage r_velocityControl = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  
  // Position Control Settings
  private PositionVoltage l_positionControl = new PositionVoltage(0, 0, false, 0, 1, false, false, false);
  private PositionVoltage r_positionControl = new PositionVoltage(0, 0, false, 0, 1, false, false, false);

  // Status Varibles
  private double l_position, r_position, l_velocity, r_velocity;

  public Drive() {
    // Config Motors
    var leftConfig = new TalonFXConfiguration();
    var rightConfig = new TalonFXConfiguration();
    
    // Set forward as positive
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  // As seen faceing motor output shaft
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //#region PID Values  - check feed forward values
    // Slot 0 - Velocity Control
    leftConfig.Slot0.kP = k_DRIVE.KP_0;
    leftConfig.Slot0.kI = k_DRIVE.KI_0;
    leftConfig.Slot0.kD = k_DRIVE.KD_0;
    leftConfig.Slot0.kV = k_DRIVE.KF_0;
    
    rightConfig.Slot0.kP = k_DRIVE.KP_0;
    rightConfig.Slot0.kI = k_DRIVE.KI_0;
    rightConfig.Slot0.kD = k_DRIVE.KD_0;
    rightConfig.Slot0.kV = k_DRIVE.KF_0;

    // Slot 1 - Position Control
    leftConfig.Slot1.kP = k_DRIVE.KP_1;
    leftConfig.Slot1.kI = k_DRIVE.KI_1;
    leftConfig.Slot1.kD = k_DRIVE.KD_1;
    leftConfig.Slot1.kV = k_DRIVE.KF_1;
    
    rightConfig.Slot1.kP = k_DRIVE.KP_1;
    rightConfig.Slot1.kI = k_DRIVE.KI_1;
    rightConfig.Slot1.kD = k_DRIVE.KD_1;
    rightConfig.Slot1.kV = k_DRIVE.KF_1;
    //#endregion


    // Apply Configs
    f500_fl_drive.getConfigurator().apply(leftConfig);
    f500_rl_drive.getConfigurator().apply(leftConfig);
    f500_fr_drive.getConfigurator().apply(rightConfig);
    f500_rr_drive.getConfigurator().apply(rightConfig);

    // Set rear Motors to follow the front
    f500_rl_drive.setControl(new Follower(f500_fl_drive.getDeviceID(), false));
    f500_rr_drive.setControl(new Follower(f500_fr_drive.getDeviceID(), false));

    f500_fl_drive.setPosition(0);
    f500_fr_drive.setPosition(0);
    
    l_position = f500_fl_drive.getPosition().getValue();
    r_position = f500_fr_drive.getPosition().getValue();
    l_velocity = f500_fl_drive.getVelocity().getValue();
    r_velocity = f500_fr_drive.getVelocity().getValue();


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    l_position = f500_fl_drive.getPosition().getValue();
    r_position = f500_fr_drive.getPosition().getValue();
    l_velocity = f500_fl_drive.getVelocity().getValue();
    r_velocity = f500_fr_drive.getVelocity().getValue();
  }

  public void RadiusDrive(double pwr, double turn){
    // Turning differential is calculated to drive a given turning radius, Positive direction to the right, positiive must be positive
    
    double l_sca = 1 - (Math.abs(turn)-turn);
    double r_sca = 1 - (Math.abs(turn)+turn);
    // set velocity setpoint for each side
    f500_fl_drive.setControl(l_velocityControl.withVelocity(pwr * l_sca * k_DRIVE.MAX_SPEED_MTR));
    f500_fr_drive.setControl(r_velocityControl.withVelocity(pwr * r_sca * k_DRIVE.MAX_SPEED_MTR));
    
  }

  public void PositionDrive(double right_target, double left_target){
    // Move drive by incremental target (in feet)

    double abs_left_target = l_position + (left_target * k_DRIVE.FT_TO_MROT);    // target in motor rotations
    double abs_right_target = r_position + (right_target * k_DRIVE.FT_TO_MROT);  // target in motor rotations

    // Set the position setpoint for each side
    f500_fl_drive.setControl(l_positionControl.withVelocity(abs_left_target));
    f500_fr_drive.setControl(r_positionControl.withVelocity(abs_right_target));
  }

   public void holdendrive (double speed,double angle ){
    // Limit Motor values to -1 to 1
    double left_motor_value = MathUtil.clamp((speed+angle), -1, 1);
    double right_motor_value = MathUtil.clamp((speed-angle), -1, 1);

    // Set Motor Speed Values
    f500_fl_drive.setControl(l_velocityControl.withVelocity(left_motor_value*k_DRIVE.MAX_SPEED_MTR));
    f500_fr_drive.setControl(r_velocityControl.withVelocity(right_motor_value*k_DRIVE.MAX_SPEED_MTR));
   }
   







}
