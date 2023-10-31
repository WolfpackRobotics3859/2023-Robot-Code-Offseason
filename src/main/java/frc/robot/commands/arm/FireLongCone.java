// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class FireLongCone extends CommandBase {
  private Arm mArm;

  /** Creates a new FirePositionOne. */
  public FireLongCone(Arm arm) {
    addRequirements(arm);
    mArm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // Intentionally Empty
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Intentionally Empty
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // Intentionally Empty
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mArm.runSingleSegmentThrow(SmartDashboard.getNumber("Arm Motion 1 Speed", 0.92), SmartDashboard.getNumber("Arm Motion 1 Ending Position", 300));
  }
}
