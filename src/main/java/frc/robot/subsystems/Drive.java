// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.k_DRIVE;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  // Declare Varibles here i.e. Speed controllers
  private TalonFX f500_fr_drive = new TalonFX(k_DRIVE.MotorID_FR);
  private TalonFX f500_rr_drive = new TalonFX(k_DRIVE.MotorID_RR);
  private TalonFX f500_fl_drive = new TalonFX(k_DRIVE.MotorID_FL);
  private TalonFX f500_rl_drive = new TalonFX(k_DRIVE.MotorID_RL);

  private MotionMagicVelocityVoltage l_velocityControl = new MotionMagicVelocityVoltage(0, k_DRIVE.MM_ACCEL, false, 0, 0, false, false, false);
  private MotionMagicVelocityVoltage r_velocityControl = new MotionMagicVelocityVoltage(0, k_DRIVE.MM_ACCEL, false, 0, 0, false, false, false);

  // Motion Magic Postion Settings
  private MotionMagicVoltage l_positionControl = new MotionMagicVoltage(0, false, 0, 1, true, false, false);
  private MotionMagicVoltage r_positionControl = new MotionMagicVoltage(0, false, 0, 1, true, false, false);

  // Status Varibles
  private double  l_position = 0, 
                  r_position = 0, 
                  l_velocity = 0, 
                  r_velocity = 0, 
                  abs_left_target, 
                  abs_right_target;

  public Drive() {
    // Config Motors
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    
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
    
    // Motion Magic Position Setting
    
    leftConfig.MotionMagic.MotionMagicCruiseVelocity = k_DRIVE.MM_CRUISE;
    leftConfig.MotionMagic.MotionMagicAcceleration = k_DRIVE.MM_POS_ACCEL;
    leftConfig.MotionMagic.MotionMagicJerk = k_DRIVE.MM_JERK;

    rightConfig.MotionMagic.MotionMagicCruiseVelocity = k_DRIVE.MM_CRUISE;
    rightConfig.MotionMagic.MotionMagicAcceleration = k_DRIVE.MM_POS_ACCEL;
    rightConfig.MotionMagic.MotionMagicJerk = k_DRIVE.MM_JERK;

    // Apply Configs
    f500_fl_drive.getConfigurator().apply(leftConfig);
    f500_rl_drive.getConfigurator().apply(leftConfig);
    f500_fr_drive.getConfigurator().apply(rightConfig);
    f500_rr_drive.getConfigurator().apply(rightConfig);

    // Neutral Mode
    f500_fl_drive.setNeutralMode(NeutralModeValue.Coast);
    f500_fr_drive.setNeutralMode(NeutralModeValue.Coast);
    f500_rl_drive.setNeutralMode(NeutralModeValue.Coast);
    f500_rr_drive.setNeutralMode(NeutralModeValue.Coast);
    
    // Set rear Motors to follow the front
    f500_rl_drive.setControl(new Follower(f500_fl_drive.getDeviceID(), false));
    f500_rr_drive.setControl(new Follower(f500_fr_drive.getDeviceID(), false));

    f500_fl_drive.setPosition(0);
    f500_fr_drive.setPosition(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update Speed & position Variables
    l_position = f500_fl_drive.getPosition().getValueAsDouble();
    r_position = f500_fr_drive.getPosition().getValueAsDouble();
    l_velocity = f500_fl_drive.getVelocity().getValueAsDouble();
    r_velocity = f500_fr_drive.getVelocity().getValueAsDouble();

    // Update Dashboard
    SmartDashboard.putNumber("Left Velocity", l_velocity / k_DRIVE.FT_TO_MROT);
    SmartDashboard.putNumber("Right Velocity", r_velocity / k_DRIVE.FT_TO_MROT);
    SmartDashboard.putNumber("Left Position", l_position / k_DRIVE.FT_TO_MROT);
    SmartDashboard.putNumber("Right Position", r_position / k_DRIVE.FT_TO_MROT);
    SmartDashboard.putNumber("Left Target", abs_left_target / k_DRIVE.FT_TO_MROT);
    SmartDashboard.putNumber("Right Target", abs_right_target / k_DRIVE.FT_TO_MROT);

  }

  public void PositionDrive(double right_target, double left_target){
    // Move drive by incremental target (in feet)

    abs_left_target = l_position + (left_target * k_DRIVE.FT_TO_MROT);    // target in motor rotations
    abs_right_target = r_position + (right_target * k_DRIVE.FT_TO_MROT);  // target in motor rotations

    // Set the position setpoint for each side
    f500_fl_drive.setControl(l_positionControl.withPosition(abs_left_target));
    f500_fr_drive.setControl(r_positionControl.withPosition(abs_right_target));
  }

  public boolean PostionOnTarget(){
    boolean left = (Math.abs(abs_left_target-l_position) < k_DRIVE.POS_ERR);
    boolean right = (Math.abs(abs_right_target-r_position) < k_DRIVE.POS_ERR);
    boolean speed = ((l_velocity < 10) && (r_velocity < 10));

    return ((left && right) && (speed));
  }

  public void holdendrive (double speed,double angle ){
    // Limit Motor values to -1 to 1
    double left_motor_value = MathUtil.clamp((speed+angle), -1, 1);
    double right_motor_value = MathUtil.clamp((speed-angle), -1, 1);

    // Set Motor Speed Values
    f500_fl_drive.setControl(l_velocityControl.withVelocity(left_motor_value*k_DRIVE.MAX_SPEED_MTR));
    f500_fr_drive.setControl(r_velocityControl.withVelocity(right_motor_value*k_DRIVE.MAX_SPEED_MTR));
  }

  // Lock the Drive Wheels
  public void PositionLock(){
    f500_fl_drive.setControl(l_positionControl.withPosition(l_position));
    f500_fr_drive.setControl(r_positionControl.withPosition(r_position));
  }
   
}
