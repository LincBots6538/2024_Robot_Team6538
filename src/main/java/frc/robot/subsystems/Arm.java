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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.k_ARM;
import frc.robot.Constants.k_WRIST;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX f500_Left = new TalonFX(k_ARM.MotorID_L);
  private final TalonFX f500_Right = new TalonFX(k_ARM.MotorID_R);

  private final TalonSRX p775_Wrist = new TalonSRX(k_WRIST.MotorID_775);

  private double  armSP,        // Desired Arm Set Point
                  wristSP,      // Desired Wrist Set Point
                  armSP_cur,    // Current Arm Set Point
                  wristSP_cur,  // Current Wrist Set Point
                  pos,          // Current Arm Position
                  wrist_pos;    // Current Wrist Postion
  private boolean armPAUSED = false,
                  wristPAUSED = false;

  private MotionMagicVoltage arm_MMconfig = new MotionMagicVoltage(0, false, 0, 0, false, false, false);


  public Arm() {
    
    // Set up Arm Motor
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    // Set Direction for positive input
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Current Limiting
    leftConfig.CurrentLimits.StatorCurrentLimit = k_ARM.CURRENT_LIMIT_NORMAL;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    rightConfig.CurrentLimits.StatorCurrentLimit = k_ARM.CURRENT_LIMIT_NORMAL;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // PID Coefficients
    leftConfig.Slot0.kP = k_ARM.KP_0;
    leftConfig.Slot0.kI = k_ARM.KI_0;
    leftConfig.Slot0.kD = k_ARM.KD_0;
  

    // Gravity Feed Forward
    leftConfig.Slot0.kG = k_ARM.KF_0;
    leftConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    leftConfig.Feedback.SensorToMechanismRatio = k_ARM.TGR;
    


    // Set motor Neutral Mode - Coast/Brake    
    f500_Left.setNeutralMode(NeutralModeValue.Brake);
    f500_Right.setNeutralMode(NeutralModeValue.Brake);
    // SmartDashboard.putString("description",f500_Left.getDescription());
   
    // Motion Magic Setup
    leftConfig.MotionMagic.MotionMagicCruiseVelocity = k_ARM.ARM_CRUISE;  // 1 second to 90 deg
    leftConfig.MotionMagic.MotionMagicAcceleration = k_ARM.ARM_ACCEL;     // Time to reach velocity
    leftConfig.MotionMagic.MotionMagicJerk = k_ARM.ARM_JERK;              // Time to reach accel
    

    
   
    // Apply Configurations
    f500_Left.getConfigurator().apply(leftConfig);
    f500_Right.getConfigurator().apply(rightConfig);

    // Set Right to Follow Left
    f500_Right.setControl(new Follower(f500_Left.getDeviceID(), true));

    // Define Starting Position
    f500_Left.setPosition(k_ARM.INIT_POS);
    

    // Set up Wrist Motor
    p775_Wrist.configFactoryDefault();
    p775_Wrist.setNeutralMode(NeutralMode.Brake);
    p775_Wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    p775_Wrist.setSensorPhase(false);   // Set so positive output drives increasing sensor values
    p775_Wrist.setInverted(false);           // Set so positive movement is up from deployed state

    // PID values
    p775_Wrist.config_kP(0, k_WRIST.KP_0);
    p775_Wrist.config_kI(0, k_WRIST.KI_0);
    p775_Wrist.config_kD(0, k_WRIST.KD_0);
    p775_Wrist.config_kF(0, k_WRIST.KF_0);

    // PID Allowable Error & Peak Output limit
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
    // Update Position Varibles 
    pos=f500_Left.getPosition().getValue()*360.0;
    wrist_pos=p775_Wrist.getSelectedSensorPosition()/k_WRIST.WRIST_DEG2TIC;
    // get Current target postions
    armSP_cur = arm_MMconfig.Position * 360.0;
    wristSP_cur = p775_Wrist.getClosedLoopTarget()/k_WRIST.WRIST_DEG2TIC;

    // Update Dashboard
    SmartDashboard.putNumber("armposition",pos);
    SmartDashboard.putNumber("wrist position",wrist_pos);
    SmartDashboard.putNumber("arm set point", armSP_cur);
    SmartDashboard.putNumber("wrist set point", wristSP_cur);
  
    //#region Crash Protection

    // Invalid Positions
    // If set Point would crash, stop
    if ((armSP < 38) && (wristSP < 0)){   
      armSP = pos;
      wristSP = wrist_pos;
      Move2Angle(armSP);
      MoveWrist(wristSP);
    }

    // If Arm is to far back = Stop
    if(pos > 95){
      armSP = pos;
      wristSP = wrist_pos;
      Move2Angle(armSP);
      MoveWrist(wristSP);
    }


    // Move Order
    // While Arm is low & wrist set point is negative, Set wrist positive
    if (pos < 38 && wristSP_cur < 0){
      wristPAUSED = true;
      p775_Wrist.set(TalonSRXControlMode.MotionMagic, 5 * k_WRIST.WRIST_DEG2TIC);
    }
    // While Wrist is negative & Arm wants to go Low, Move wrist
    if (wrist_pos < -2 && armSP_cur < 38){
      armPAUSED = true;
      f500_Left.setControl(arm_MMconfig.withPosition(pos / 360.0));
    }

    // If Arm is done, Unpause Wrist
    if (wristPAUSED && IsArmStopped()) {
      MoveWrist(wristSP);
      wristPAUSED = false;
    }
    // If Wrist is done, Unpause Arm
    if (armPAUSED && IsWristStopped()) {
      Move2Angle(armSP);
      armPAUSED = false;
      
    }
    //#endregion
  }

  /** Moves the Arm Shoulder Joint
   *  @param angle - Angle to move, in degrees */
  public void Move2Angle(double angle){
    f500_Left.setControl(arm_MMconfig.withPosition(angle/360.0));
    armSP=angle;  
  }

  /** Moves the Wrist Joint
   *  @param angle - Angle to move, in degrees */
  public void MoveWrist(double angle){
    p775_Wrist.set(TalonSRXControlMode.MotionMagic, angle * k_WRIST.WRIST_DEG2TIC);
    wristSP=angle;
  }

  public void wristDC(double pctOutput){
    p775_Wrist.set(TalonSRXControlMode.PercentOutput, pctOutput);
  }

  public void shoulderDC(double pctOutput){
    f500_Left.set(pctOutput);
  }
  public void holden(){
    //do nothing
  }

  public boolean IsStopped(){
    boolean arm = (Math.abs(f500_Left.getVelocity().getValueAsDouble()) * 360.0 < 1.0);
    boolean wrist = (Math.abs(p775_Wrist.getSelectedSensorVelocity() / k_WRIST.WRIST_DEG2TIC) < 1.0);
    return (arm && wrist);
  }

  private boolean IsArmStopped(){
    //boolean arm = (Math.abs(f500_Left.getVelocity().getValueAsDouble()) * 360.0 < 1.0);
    boolean arm = (Math.abs(armSP-pos) < 5);
    return (arm);
  }

  private boolean IsWristStopped(){
    //boolean wrist = (Math.abs(p775_Wrist.getSelectedSensorVelocity() / k_WRIST.WRIST_DEG2TIC) < 1.0);
    boolean wrist = (Math.abs(wristSP-wrist_pos) < 5);
    return (wrist);
  }

  public boolean IsAtTarget(){
    return ((Math.abs(armSP-pos) < 5) && (Math.abs(wristSP-wrist_pos) < 5));
  }
  

  public double get_wrist_position() {
    return wrist_pos;
  }

  public double get_arm_position() {
    return pos;
  }

  public void resetSensors(){
    f500_Left.setPosition(k_ARM.INIT_POS);
    p775_Wrist.setSelectedSensorPosition(k_WRIST.INIT_POS);
  }

  public boolean Arm10(){
    if (pos > 10){
      return true;
    } else {
      return false;
    }

  }


}
