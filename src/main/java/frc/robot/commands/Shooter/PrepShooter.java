// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.k_INTAKE;
import frc.robot.commands.Intake.SetRoller;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepShooter extends SequentialCommandGroup {
  /** Creates a new PrepShooter. */
  public PrepShooter(shooter shooter_sys, Intake intake_sys, double speedSP) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetRoller(intake_sys, k_INTAKE.PWR_OUT),
      new WaitCommand(0.1),
      new SetRoller(intake_sys, 0),
      new SetShooter(shooter_sys, speedSP)
      );
  }
}
