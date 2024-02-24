// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.k_CONTROLLERS;
import frc.robot.commands.AutoDrvStraight;
import frc.robot.commands.SetArmPosistions;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.complexarm;
import frc.robot.commands.complexwrist;
import frc.robot.commands.roller;
import frc.robot.commands.simplearmdrive;
import frc.robot.commands.simpleshooter;
import frc.robot.commands.simplewrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
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
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(k_CONTROLLERS.DRIVE_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Default Drive Command
    sys_Drive.setDefaultCommand(new TeleopDrive(sys_Drive,
        m_driverController::getRightTriggerAxis,
        m_driverController::getLeftTriggerAxis,
        m_driverController::getRightX));



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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //m_driverController.x().whileTrue(new simplearmdrive(sys_Arm,0.1));
    //m_driverController.a().whileTrue(new simplearmdrive(sys_Arm,-0.1));
    //m_driverController.y().whileTrue(new simplewrist(sys_Arm,0.2));
    //m_driverController.b().whileTrue(new simplewrist(sys_Arm,-0.2));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    //m_driverController.x().whileTrue(new complexarm(sys_Arm,45));
    //m_driverController.leftBumper().whileTrue(new complexwrist(sys_Arm,90));
    
    //m_driverController.x().whileTrue(new complexarm(sys_Arm,10));
    // old button bindings might be possibly used for seperate manipulator controller 
    

    m_driverController.a().onTrue(new SetArmPosistions(sys_Arm, 0)); // go to home position
    m_driverController.x().onTrue(new SetArmPosistions(sys_Arm, 1)); // go to intake position
    m_driverController.y().onTrue(new SetArmPosistions(sys_Arm,2)); //  go to speaker position 
    m_driverController.b().onTrue(new SetArmPosistions(sys_Arm,3)); // go to amp position 
    m_driverController.back().onTrue(new SetArmPosistions(sys_Arm,4)); // go to climb position
    m_driverController.rightBumper().whileTrue(new simpleshooter(sys_Shooter, -10000));
    m_driverController.leftBumper().whileTrue(new roller(sysIntake,-0.35));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    var cmd = new AutoDrvStraight(sys_Drive,5.0);
    return cmd;
  }
}
