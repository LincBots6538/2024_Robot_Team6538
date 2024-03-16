// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooter extends InstantCommand {
  private shooter sys_shooter;
  private double speed;

  public SetShooter(shooter subsystem, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    sys_shooter = subsystem;
    speed = RPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_shooter.set_speed(speed);
  }
}
