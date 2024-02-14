// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.k_ARM;
import frc.robot.Constants.k_WRIST;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX f500_Left = new TalonFX(k_ARM.MotorID_L);
  private final TalonFX f500_Right = new TalonFX(k_ARM.MotorID_R);

  private final TalonSRX p775_Wrist = new TalonSRX(k_WRIST.MotorID_775);

  private PositionVoltage positionControl = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
  
  public Arm() {
    
    // Set up Arm Motor
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    leftConfig.CurrentLimits.StatorCurrentLimit = k_ARM.CURRENT_LIMIT_NORMAL;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    rightConfig.CurrentLimits.StatorCurrentLimit = k_ARM.CURRENT_LIMIT_NORMAL;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    f500_Left.setNeutralMode(NeutralModeValue.Brake);
    f500_Right.setNeutralMode(NeutralModeValue.Brake);

    f500_Left.getConfigurator().apply(leftConfig);
    f500_Right.getConfigurator().apply(rightConfig);

    f500_Right.setControl(new Follower(f500_Left.getDeviceID(), true));

    f500_Left.setPosition(k_ARM.INIT_POS);

    

    // Set up Wrist Motor
    p775_Wrist.configFactoryDefault();
    p775_Wrist.setNeutralMode(NeutralMode.Brake);
    p775_Wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    p775_Wrist.setSensorPhase(true);   // Set so positive output drives increasing sensor values
    p775_Wrist.setInverted(true);           // Set so positive movement is up from deployed state

    // PID values
    p775_Wrist.config_kP(0, k_WRIST.KP_0);
    p775_Wrist.config_kI(0, k_WRIST.KI_0);
    p775_Wrist.config_kD(0, k_WRIST.KD_0);
    p775_Wrist.config_kF(0, k_WRIST.KF_0);

    p775_Wrist.configAllowableClosedloopError(0, k_WRIST.PID_ERR);
    p775_Wrist.configClosedLoopPeakOutput(0, k_WRIST.PID_OUTPUT);

    p775_Wrist.configMotionCruiseVelocity(k_WRIST.PID_CRUISE_SPD);
    p775_Wrist.configMotionAcceleration(k_WRIST.PID_CRUISE_ACC);
    p775_Wrist.configMotionSCurveStrength(k_WRIST.MM_STRENGTH);

    p775_Wrist.selectProfileSlot(0, 0);
    p775_Wrist.setSelectedSensorPosition(k_WRIST.INIT_POS);   // 2023 Set sensor positions 0.2 sec after first loop

    // Current Limits
    p775_Wrist.configPeakCurrentLimit(0);
    p775_Wrist.configContinuousCurrentLimit(k_WRIST.CURRENT_LIMIT_NORMAL);
    p775_Wrist.enableCurrentLimit(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Move2Angle(double angle){
    f500_Left.setControl(positionControl.withPosition(angle * k_ARM.DEG2MROT));
  }

  public void MoveWrist(double angle){
    p775_Wrist.set(TalonSRXControlMode.Position, angle * k_WRIST.WRIST_DEG2TIC);
  }

  public void wristDC(double pctOutput){
    p775_Wrist.set(TalonSRXControlMode.PercentOutput, pctOutput);
  }

  public void shoulderDC(double pctOutput){
    f500_Left.set(pctOutput);
  }
}