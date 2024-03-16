// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drive;

public class AutoDrvStraight extends Command {
  /** Creates a new AutoDrvStraight. */
  private Drive sys_Drive;
  private double dis;

  public AutoDrvStraight(Drive sysDrive, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_Drive = sysDrive;
    dis = distance;
    addRequirements(sysDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_Drive.PositionDrive(dis, dis);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_Drive.PostionOnTarget();
  }
}
