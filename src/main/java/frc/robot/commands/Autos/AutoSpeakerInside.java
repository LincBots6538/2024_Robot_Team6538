// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.k_SETPOINTS;
import frc.robot.Constants.k_SHOOTER;
import frc.robot.commands.Arm.ArmPose;
import frc.robot.commands.Drive.AutoDrvStraight;
import frc.robot.commands.Drive.AutoDrvTurn;
import frc.robot.commands.Intake.SetRoller;
import frc.robot.commands.Shooter.PrepShooter;
import frc.robot.commands.Shooter.SetShooter;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.Constants.k_INTAKE;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSpeakerInside extends SequentialCommandGroup {
  /** Creates a new AutoSpeakerInside. */
  public AutoSpeakerInside(Drive drive_sys, Arm arm_sys, Intake intake_sys, shooter shooter_sys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double side = 1;
    Alliance alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Blue)  side = -1;

    addCommands(
      // Ready Arm to Shoot
      new ParallelCommandGroup(
        new ArmPose(arm_sys, k_SETPOINTS.ARM_SPEAKER_BW, k_SETPOINTS.WRIST_SPEAKER_BW),
        new PrepShooter(shooter_sys, intake_sys, k_SHOOTER.SPEAKER_SPEED)
      ),
      // Keep arm Stable & Shoot
      new WaitCommand(0.1),
      new Shoot(shooter_sys, intake_sys),
      new WaitCommand(.2),
      // Drive away & Store Arm
      new ParallelCommandGroup(
        new AutoDrvStraight(drive_sys, 0.8),
        new SetShooter(shooter_sys, 0),
        new SetRoller(intake_sys, 0),
        new ArmPose(arm_sys, k_SETPOINTS.ARM_HOME, k_SETPOINTS.WRIST_HOME)
      ),
      // Turn to Note & Acquire
      new ParallelCommandGroup(    
        // Drive             
        new SequentialCommandGroup(
          new AutoDrvTurn(drive_sys, side * -70),
          new WaitCommand(.5),
          new AutoDrvStraight(drive_sys, 5.6)
        ),
        // Move Arm to Intake & turn on Intake
        new ArmPose(arm_sys, k_SETPOINTS.ARM_INTAKE, k_SETPOINTS.WRIST_INTAKE),
        new SetRoller(intake_sys, k_INTAKE.PWR_FREE)
      ),
      // Pause to ensure note is in the intake
      new WaitCommand(0.1),
      new SetRoller(intake_sys, 0),
      // Drive Back & Turn, while moving arm home and preping note.
      new ParallelCommandGroup(    
        // Drive             
        new SequentialCommandGroup(
          new AutoDrvStraight(drive_sys, -5.6),
          new AutoDrvTurn(drive_sys, side * 70)
        ),
        // Move Arm Home & Make Sure Note is all the way in
        new ArmPose(arm_sys, k_SETPOINTS.ARM_HOME, k_SETPOINTS.WRIST_HOME),
        new SequentialCommandGroup(
          new WaitCommand(0.1),
          new SetRoller(intake_sys, k_INTAKE.PWR_FREE),
          new WaitCommand(0.1),
          new SetRoller(intake_sys, k_INTAKE.PWR_OUT),
          new WaitCommand(0.05),
          new SetRoller(intake_sys, k_INTAKE.PWR_FREE),
          new WaitCommand(0.1),
          new SetRoller(intake_sys, 0)
        )
      ),
      // Finish Driving back to the Speaker, Ready arm to shoot
      new ParallelCommandGroup(           
        new AutoDrvStraight(drive_sys, -0.8),
        new ArmPose(arm_sys, k_SETPOINTS.ARM_SPEAKER_BW, k_SETPOINTS.WRIST_SPEAKER_BW),
        new PrepShooter(shooter_sys, intake_sys, k_SHOOTER.SPEAKER_SPEED)
      ),
      // Keep arm Stable & Shoot
      new WaitCommand(0.1),
      new Shoot(shooter_sys, intake_sys),
      new WaitCommand(.2),
      // Drive Away to Source Side 
      new ParallelCommandGroup(
        new AutoDrvStraight(drive_sys, 13.75),
        new SetShooter(shooter_sys, 0),
        new SetRoller(intake_sys, 0),
        new ArmPose(arm_sys, k_SETPOINTS.ARM_HOME, k_SETPOINTS.WRIST_HOME)
      )
    );
  }
}
