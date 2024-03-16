// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.k_SETPOINTS;
import frc.robot.commands.Arm.ArmPose;
import frc.robot.commands.Drive.drivelock;
import frc.robot.commands.Intake.SetRoller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(Drive drive_sys, Arm arm_sys, Intake intake_sys, shooter shooter_sys, double ArmPOS, double WristPOS, double ShooterSPD) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.race(
        new drivelock(drive_sys),
        new SequentialCommandGroup(
          new PrepShooter(shooter_sys, intake_sys, ShooterSPD),
          new ArmPose(arm_sys, ArmPOS, WristPOS),
          new Shoot(shooter_sys, intake_sys),
          new WaitCommand(0.1)
        )
      ),
      new SetShooter(shooter_sys, 0),
      new SetRoller(intake_sys, 0),
      new ArmPose(arm_sys, k_SETPOINTS.ARM_HOME, k_SETPOINTS.WRIST_HOME)
    );
  }
}
