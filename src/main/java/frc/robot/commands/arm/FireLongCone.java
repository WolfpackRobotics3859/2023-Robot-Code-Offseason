// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class FireLongCone extends Command 
{
  private Arm mArm;
  private double mSeg1Speed, mSeg2Speed, mSeg1End, mSeg2End;

  public FireLongCone(Arm arm) 
  {
    addRequirements(arm);
    mArm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    mSeg1Speed = SmartDashboard.getNumber("Arm Motion 1 Segment 1 Speed", 1);
    mSeg2Speed = SmartDashboard.getNumber("Arm Motion 1 Segment 2 Speed", 0.3);
    mSeg1End = SmartDashboard.getNumber("Arm Motion 1 Segment 1 Ending Position", 300);
    mSeg2End = SmartDashboard.getNumber("Arm Motion 1 Segment 2 Ending Position", 700);
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
  public boolean isFinished() 
  {
    return mArm.runDoubleSegmentThrow(mSeg1Speed, mSeg1End, mSeg2Speed, mSeg2End);
  }
}
