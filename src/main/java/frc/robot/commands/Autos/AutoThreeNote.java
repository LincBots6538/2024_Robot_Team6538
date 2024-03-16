// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.k_INTAKE;
import frc.robot.Constants.k_SETPOINTS;
import frc.robot.Constants.k_SHOOTER;
import frc.robot.commands.Arm.ArmPose;
import frc.robot.commands.Drive.AutoDrvStraight;
import frc.robot.commands.Drive.AutoDrvTurn;
import frc.robot.commands.Intake.SetRoller;
import frc.robot.commands.Shooter.PrepShooter;
import frc.robot.commands.Shooter.SetShooter;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoThreeNote extends SequentialCommandGroup {
  /** Creates a new AutoThreeNote. */
  public AutoThreeNote(Drive drive_sys, Arm arm_sys, Intake intake_sys, shooter shooter_sys) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double side = 1;
    Alliance alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Blue)  side = -1;

    addCommands(
      // Ready Arm to Shoot
      new ParallelCommandGroup(
        new ArmPose(arm_sys, k_SETPOINTS.ARM_SPEAKER_LOW, k_SETPOINTS.WRIST_SPEAKER_LOW),
        new PrepShooter(shooter_sys, intake_sys, k_SHOOTER.SPEAKER_SPEED)
      ),
      // Keep arm Stable & Shoot
      new WaitCommand(0.1),
      new Shoot(shooter_sys, intake_sys),
      new WaitCommand(.2),
      // Turn Rollers off and Drive Forward
      new SetRoller(intake_sys, k_INTAKE.PWR_FREE),
      new SetShooter(shooter_sys, 0),
      new ArmPose(arm_sys, k_SETPOINTS.ARM_INTAKE, k_SETPOINTS.WRIST_INTAKE),
      new AutoDrvStraight(drive_sys, 3.8),
      new WaitCommand(0.1),
      new SetRoller(intake_sys, 0),

      // Pick up and Shoot
      new ParallelCommandGroup(
        new ArmPose(arm_sys, k_SETPOINTS.ARM_SPEAKER_LOW, k_SETPOINTS.WRIST_SPEAKER_MID),
        new PrepShooter(shooter_sys, intake_sys, k_SHOOTER.SPEAKER_SPEED)
      ),
      // Keep arm Stable & Shoot
      new WaitCommand(0.1),
      new Shoot(shooter_sys, intake_sys),
      new WaitCommand(.2),
      // Turn Rollers off and Drive Forward
      new SetRoller(intake_sys, 0),
      new SetShooter(shooter_sys, 0),
      // Drive to Next Note
      new AutoDrvTurn(drive_sys, side * -56.0),
      new AutoDrvStraight(drive_sys, -5.8),
      new AutoDrvTurn(drive_sys, side * 56.0),
      // Drive Forward & Intake Note
      new SetRoller(intake_sys, k_INTAKE.PWR_FREE),
      new ArmPose(arm_sys, k_SETPOINTS.ARM_INTAKE, k_SETPOINTS.WRIST_INTAKE),
      new AutoDrvStraight(drive_sys, 2.2),
      new WaitCommand(0.1),
      new SetRoller(intake_sys, 0),

      // Drive Back enough to rotate, and Aim at Speaker
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new AutoDrvStraight(drive_sys, -.5),
          new AutoDrvTurn(drive_sys, side * 34.7)
        ),
        new ArmPose(arm_sys, k_SETPOINTS.ARM_SPEAKER_LOW, k_SETPOINTS.WRIST_SPEAKER_MID),
        new PrepShooter(shooter_sys, intake_sys, k_SHOOTER.SPEAKER_SPEED)
      ),
      // Keep arm Stable & Shoot
      new WaitCommand(0.1),
      new Shoot(shooter_sys, intake_sys),
      new WaitCommand(0.2),

      // Turn rollers off, Send Arm Home
      new SetRoller(intake_sys, 0),
      new SetShooter(shooter_sys, 0),
      new ArmPose(arm_sys, k_SETPOINTS.ARM_HOME, k_SETPOINTS.WRIST_HOME)
    );
  }
}
