// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.k_INTAKE;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  private shooter sys_shooter;
  private Intake sys_intake;
  
  public Shoot(shooter shooter, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
    sys_intake = intake;
    sys_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_intake.Roller(k_INTAKE.PWR_FREE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_shooter.OnTarget();
  }
}
