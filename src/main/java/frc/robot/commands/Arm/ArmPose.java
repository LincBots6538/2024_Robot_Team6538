// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmPose extends Command {
  /** Creates a new ArmPose. */
  private Arm sys_arm;
  private double armSP, wristSP;

  public ArmPose(Arm subsystem, double arm, double wrist) {
    addRequirements(subsystem);

    sys_arm = subsystem;
    armSP = arm;
    wristSP = wrist;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    sys_arm.Move2Angle(armSP);
    sys_arm.MoveWrist(wristSP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      sys_arm.Move2Angle(sys_arm.get_arm_position());
      sys_arm.MoveWrist(sys_arm.get_wrist_position());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_arm.IsAtTarget();
  }
}
