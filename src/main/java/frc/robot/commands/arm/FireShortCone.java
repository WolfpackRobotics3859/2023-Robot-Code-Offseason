// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class FireShortCone extends CommandBase {
  private Arm mArm;
  private double seg1Speed, seg2Speed, seg1End, seg2End;

  /** Creates a new FirePositionOne. */
  public FireShortCone(Arm arm) {
    addRequirements(arm);
    mArm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    seg1Speed = SmartDashboard.getNumber("Arm Motion 2 Segment 1 Speed", 0.2);
    seg2Speed = SmartDashboard.getNumber("Arm Motion 2 Segment 2 Speed", 0.4);
    seg1End = SmartDashboard.getNumber("Arm Motion 2 Segment 1 Ending Position", 300);
    seg2End = SmartDashboard.getNumber("Arm Motion 2 Segment 2 Ending Position", 400);
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
    return mArm.runDoubleSegmentThrow(seg1Speed, seg1End, seg2Speed, seg2End);
  }
}
