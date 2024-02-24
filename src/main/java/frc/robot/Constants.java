// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class k_CONTROLLERS {
    public static final int DRIVE_PORT = 0;
    public static final double JOYSTICK_DEADBAND = 0.1;
    

  }
  public static class k_DRIVE {
    // Drive Motor CAN IDs
    public static final int MotorID_FR = 1;
    public static final int MotorID_RR = 0;
    public static final int MotorID_FL = 3;
    public static final int MotorID_RL = 2;

    // Mechanical Values
    public static final double GEAR_RATIO = 10.75;
    public static final double WHEEL_DIAMETER = 6.0/12.0;     // Wheel diameter in feet
    public static final double TRACK_WIDTH = 25.0/12.0;       // Chassis track width in feet
    public static final double FT_TO_MROT = GEAR_RATIO / (Math.PI * WHEEL_DIAMETER);             // Motor rotations in one foot of travel
    public static final double SPEED_EXP = 2;
    public static final double TURN_EXP = 3;
    // Calibration Values
    public static final double MAX_SPEED = 15.0;              // Max speed in ft/s (free speed max @ 10.75 ratio & 6in wheels is 15.6 ft/s)
      // Max Speed converted to Motor Speed rotations per second
    public static final double MAX_SPEED_MTR = MAX_SPEED * FT_TO_MROT;
    public static final double NOM_RADIUS = 10.0;        // The turning radius in feet represented at 50% turning input 

    // PID Constants
    // Velocity Control
    public static final double KP_0 = 0.012;
    public static final double KI_0 = 0;
    public static final double KD_0 = 0;
    public static final double KF_0 = 0.12;

    // Position Control
    public static final double KP_1 = 1.2;
    public static final double KI_1 = 0;
    public static final double KD_1 = 0;
    public static final double KF_1 = 0;
  }

  public static class k_ARM {
    // Motor CAN IDs
    public static final int MotorID_R = 4;
    public static final int MotorID_L = 5;

    // Postion PID constants
    public static final double KP_0 = 50.0;   // 2.40 should be equavalent to last year's kP, might want to start low and work up to this
    public static final double KI_0 = 0;
    public static final double KD_0 = 0;
    public static final double KF_0 = 0.8;    //Gravity Feedforward set in Arm subsystem, increase slowly (adds %output to counter gravity)

    // Arm Mechanics
    public static final double M2P = 24.0/16.0;     // Gear ratio motor to planetary input
    public static final double BB_GR = 64.0;      // Gear ratio of BaneBots Planetary
    public static final double FGR = 2.0;         // Gear ratio of Planetary output to Arm
    public static final double TGR = M2P * BB_GR * FGR;
    public static final double DEG2MROT = M2P * BB_GR * FGR / 360;    //  Motor Rotations per deg of Arm rotation
    public static final double INIT_POS = 0;    // Initial position of the arm

    public static final double CURRENT_LIMIT_NORMAL = 20;
    public static final double CURRENT_LIMIT_CLIMB = 50;

    // Motion Magic Settings
    public static final double ARM_CRUISE = 0.5;  // rot/s  - Mechanism rotations?
    public static final double ARM_CRUISE_TIME = 0.5;  // seconds to reach Cruise Speed
    public static final double ARM_ACCEL = ARM_CRUISE / (ARM_CRUISE_TIME);  
    public static final double ARM_ACCEL_TIME = 0.1;  // seconds to reach Acceleration
    public static final double ARM_JERK = ARM_ACCEL / (ARM_ACCEL_TIME);  
  }

  public static class k_WRIST {
    // Motor CAN IDs
    public static final int MotorID_775 = 7;          // 775 Motor ID

    // Wrist Mechanics
    public static final int ENCODER_CNT = 4096;
    public static final double WR_GR = 2.0;              // Gear Ratio between VP Output and intake hex shaft
    public static final double WRIST_DEG2TIC = WR_GR * ENCODER_CNT / 360.0;        // Motor ticks per degrees of Wrist rotation
    public static final double INIT_POS = 180 * WRIST_DEG2TIC;    // 0 ref is inline with Arms (or deployed state)
    
    // PID Constants
    public static final double KP_0 = 1;
    public static final double KI_0 = 0;
    public static final double KD_0 = 0;
    public static final double KF_0 = 0;
    public static final double PID_ERR = 10;
    public static final double PID_OUTPUT = 0.5;
    public static final double PID_CRUISE_SPD = 90.0 * WRIST_DEG2TIC / 10;
    public static final double PID_CRUISE_ACC = PID_CRUISE_SPD * 3;
    public static final int MM_STRENGTH = 0;     // 0-8 : 0 is trapizodal, increasing smoothing

    public static final int CURRENT_LIMIT_NORMAL = 20;

  }
  public static class k_INTAKE {
    // Motor CAN IDs
    public static final int MotorID_Rol = 6;          // Bag Motor ID
    
    // Motor Power Level
    public static final double PWR_OBSTRUCTED = 0.5;  // Roller power when note is pushing somethingn or bending
    public static final double PWR_FREE = 0.25;       // Roller power when nothing is in the way
    public static final double JERK_TIME = 0.1;
    public static final int CURRENT_LIMIT_NORMAL = 10;
    
  }

  public static class k_SHOOTER {
    public static final int MotorID_TOP = 8;
    public static final int MotorID_BOT = 9;
    
  }
}
