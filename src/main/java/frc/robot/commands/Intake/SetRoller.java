// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetRoller extends InstantCommand {
  private Intake intake;
  private double pwr;

  public SetRoller(Intake subsystem, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    intake = subsystem;
    pwr = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.Roller(pwr);
  }
}
