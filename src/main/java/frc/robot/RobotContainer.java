// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.k_CONTROLLERS;
import frc.robot.Constants.k_INTAKE;
import frc.robot.Constants.k_SETPOINTS;
import frc.robot.Constants.k_SHOOTER;
import frc.robot.commands.Arm.ArmPose;
import frc.robot.commands.Autos.AutoCenterLineStealAMP;
import frc.robot.commands.Autos.AutoDoubleSpeakerCenter;
import frc.robot.commands.Autos.AutoInsideSteal;
import frc.robot.commands.Autos.AutoSideMove;
import frc.robot.commands.Autos.AutoSpeakerInside;
import frc.robot.commands.Autos.AutoSpeakerOutside;
import frc.robot.commands.Drive.AutoDrvStraight;
import frc.robot.commands.Drive.TeleopDrive;
import frc.robot.commands.Drive.drivelock;
import frc.robot.commands.Intake.SetRoller;
import frc.robot.commands.Intake.roller;
import frc.robot.commands.Shooter.AutoShoot;
import frc.robot.commands.Shooter.PrepShooter;
import frc.robot.commands.Shooter.SetShooter;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.shooter;
import frc.robot.util.CommandNaconController;
import frc.robot.subsystems.Intake;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //#region Declare Subsystems
  private final Drive sys_Drive = new Drive();
  private final Arm sys_Arm = new Arm();
  private final shooter sys_Shooter = new shooter();
  private final Intake sysIntake = new Intake();
  //#endregion
  
  //Commands
  

  // Auto Chooser
  private SendableChooser<Command> sel_Auto = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandNaconController m_driverController =
     new CommandNaconController(k_CONTROLLERS.DRIVE_PORT);

  //private final CommandNaconController m_testController = new CommandNaconController(k_CONTROLLERS.DRIVE_PORT);

  // private final CommandXboxController m_manipController = 
  //    new CommandXboxController(k_CONTROLLERS.MANIP_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

   CameraServer.startAutomaticCapture();
   


    // Default Drive Command
    sys_Drive.setDefaultCommand(new TeleopDrive(sys_Drive,
        m_driverController::getR2Axis,
        m_driverController::getL2Axis,
        m_driverController::getRightX));

    //m_testController.getR2Axis();

    sel_Auto.setDefaultOption("Center - Double", new AutoDoubleSpeakerCenter(sys_Drive, sys_Arm, sysIntake, sys_Shooter));
    sel_Auto.addOption("Source Side - Double", new AutoSpeakerInside(sys_Drive, sys_Arm, sysIntake, sys_Shooter));
    sel_Auto.addOption("Amp Side - Double", new AutoSpeakerOutside(sys_Drive, sys_Arm, sysIntake, sys_Shooter));
    sel_Auto.addOption("Source Side - Single + Steal", new AutoInsideSteal(sys_Drive, sys_Arm, sysIntake, sys_Shooter));
    sel_Auto.addOption("Amp Side - Single + Pause", new AutoSideMove(sys_Drive, sys_Arm, sysIntake, sys_Shooter));
    sel_Auto.addOption("Amp Side - Just Steal", new AutoCenterLineStealAMP(sys_Drive, sys_Arm, sysIntake, sys_Shooter));
    sel_Auto.addOption("Drive Forward", new AutoDrvStraight(sys_Drive, 20));
    sel_Auto.addOption("Do Nothing", null);

    Shuffleboard.getTab("Auto").add(sel_Auto);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() { 
    // Drive Stop
    //Trigger t_Arm10 = new Trigger(sys_Arm::Arm10);
   // t_Arm10.whileTrue(new drivelock(sys_Drive));

    //#region DRIVE Controller
    // Arm Positions - Buttons
    m_driverController.cross().onTrue(new ParallelCommandGroup(     // Go to Home
      new SetRoller(sysIntake, 0),
      new SetShooter(sys_Shooter, 0),
      new ArmPose(sys_Arm, k_SETPOINTS.ARM_HOME, k_SETPOINTS.WRIST_HOME))); 
    m_driverController.square().onTrue(new SequentialCommandGroup(     // Go to Intake Position
      new SetShooter(sys_Shooter, 0),  
      new ArmPose(sys_Arm, k_SETPOINTS.ARM_INTAKE, k_SETPOINTS.WRIST_INTAKE),
      new SetRoller(sysIntake, k_INTAKE.PWR_FREE))); 
    m_driverController.triangle().onTrue(new SequentialCommandGroup(     // Go to Speaker / Prep Shooter
      new SetRoller(sysIntake, 0),
      new ArmPose(sys_Arm, k_SETPOINTS.ARM_SPEAKER_BW, k_SETPOINTS.WRIST_SPEAKER_BW),
      new PrepShooter(sys_Shooter, sysIntake, k_SHOOTER.SPEAKER_SPEED))); 
    m_driverController.circle().onTrue(new SequentialCommandGroup(     // Go to Amp Position
      new SetRoller(sysIntake, 0),
      new ArmPose(sys_Arm, k_SETPOINTS.ARM_AMP_FW, k_SETPOINTS.WRIST_AMP_FW))); 
    m_driverController.share().onTrue(new ParallelCommandGroup(  // Go to Climb Prep
      new SetRoller(sysIntake, 0),
      new SetShooter(sys_Shooter, 0),
      new ArmPose(sys_Arm, k_SETPOINTS.ARM_CLIMB, k_SETPOINTS.WRIST_CLIMB))); 
    m_driverController.options().onTrue(new ParallelCommandGroup(  // Go to Climb Home
      new SetRoller(sysIntake, 0),
      new SetShooter(sys_Shooter, 0),
      new ArmPose(sys_Arm, k_SETPOINTS.ARM_CLIMB_HOME, k_SETPOINTS.WRIST_CLIMB_HOME))); 
    
    // Arm Positions - Left Analog Stick
    // m_driverController.axisLessThan(CommandNaconController.Axis.kLeftY.value, -0.8).onTrue(new SequentialCommandGroup(     // Up = Dump
    //   new SetRoller(sysIntake, 0),
    //   new ArmPose(sys_Arm, k_SETPOINTS.ARM_FAR, k_SETPOINTS.WRIST_FAR),
    //   new PrepShooter(sys_Shooter, sysIntake, k_SHOOTER.SPEAKER_SPEED)));
    // m_driverController.axisLessThan(CommandNaconController.Axis.kLeftX.value, -0.8).onTrue(new ParallelCommandGroup(                 // Left = source backword
    //   new SetRoller(sysIntake, 0),
    //   new SetShooter(sys_Shooter, 0),
    //   new ArmPose(sys_Arm, k_SETPOINTS.ARM_SOURCE_BW, k_SETPOINTS.WRIST_SOURCE_BW)));
    // m_driverController.axisGreaterThan(CommandNaconController.Axis.kLeftY.value, 0.8).onTrue(new ParallelCommandGroup(                 // Down = 
    //   new SetRoller(sysIntake, 0),
    //   new SetShooter(sys_Shooter, 0),
    //   new ArmPose(sys_Arm, k_SETPOINTS.ARM_HOME, k_SETPOINTS.WRIST_HOME)));
    //m_driverController.axisGreaterThan(CommandNaconController.Axis.kLeftX.value, 0.8).onTrue(new ParallelCommandGroup(     // Right = 
    //  new SetRoller(sysIntake, 0),
     // new SetShooter(sys_Shooter, 0),
      //new ArmPose(sys_Arm, k_SETPOINTS.ARM_AMP_FW, k_SETPOINTS.WRIST_AMP_FW)));

    // Right Far
    m_driverController.axisGreaterThan(CommandNaconController.Axis.kLeftX.value, 0.8).onTrue(
      new AutoShoot(sys_Drive, sys_Arm, sysIntake, sys_Shooter, k_SETPOINTS.ARM_QUICK, k_SETPOINTS.WRIST_SPEAKER_FAR, k_SHOOTER.SPEAKER_SPEED));
    // Left
    m_driverController.axisLessThan(CommandNaconController.Axis.kLeftX.value, -0.8).onTrue(
      new AutoShoot(sys_Drive, sys_Arm, sysIntake, sys_Shooter, k_SETPOINTS.ARM_QUICK, k_SETPOINTS.WRIST_SPEAKER_LOW, k_SHOOTER.SPEAKER_SPEED));
    // Up
    m_driverController.axisLessThan(CommandNaconController.Axis.kLeftY.value, 0.8).onTrue(
      new AutoShoot(sys_Drive, sys_Arm, sysIntake, sys_Shooter, k_SETPOINTS.ARM_QUICK, k_SETPOINTS.WRIST_SPEAKER_MID, k_SHOOTER.SPEAKER_SPEED));
    // Down
    m_driverController.axisGreaterThan(CommandNaconController.Axis.kLeftY.value, 0.8).onTrue(
      new AutoShoot(sys_Drive, sys_Arm, sysIntake, sys_Shooter, k_SETPOINTS.ARM_QUICK, k_SETPOINTS.WRIST_OVER, k_SHOOTER.SPEAKER_SPEED));
    
    // L/R Bumpers
    Trigger t_shot = new Trigger(sys_Shooter::OnTarget);
    m_driverController.R1().and(t_shot).onTrue(new SetRoller(sysIntake, k_INTAKE.PWR_FREE));
    //m_driverController.R1().whileTrue(new Shoot(sys_Shooter, sysIntake));
    m_driverController.R1().onFalse(new SetRoller(sysIntake, 0));
    m_driverController.L1().whileTrue(new roller(sysIntake, k_INTAKE.PWR_FREE));

    // D-Pad
    m_driverController.povLeft().whileTrue(new roller(sysIntake, k_INTAKE.PWR_FREE*-1));
    m_driverController.povUp().onTrue(new ParallelCommandGroup(
      new SetRoller(sysIntake, 0),
      new SetShooter(sys_Shooter, 0)));

    //#endregion
    
    

    // Manip Controller - Manually Move Arm

    // m_manipController.rightBumper().toggleOnTrue(new drivelock(sys_Drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return sel_Auto.getSelected();
  }
}
