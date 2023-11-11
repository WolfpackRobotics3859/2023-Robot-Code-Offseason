// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class FireLow extends CommandBase 
{
  private Arm mArm;
  private double mSeg1Speed, mSeg2Speed, mSeg1End, mSeg2End;

  public FireLow(Arm arm) 
  {
    addRequirements(arm);
    mArm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    mSeg1Speed = SmartDashboard.getNumber("Arm Motion 4 Segment 1 Speed", 0.5);
    mSeg2Speed = SmartDashboard.getNumber("Arm Motion 4 Segment 2 Speed", 0);
    mSeg1End = SmartDashboard.getNumber("Arm Motion 4 Segment 1 Ending Position", 175);
    mSeg2End = SmartDashboard.getNumber("Arm Motion 4 Segment 2 Ending Position", 175);
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
