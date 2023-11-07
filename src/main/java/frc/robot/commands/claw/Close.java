// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class Close extends CommandBase 
{
  private Claw mClaw;

  public Close(Claw claw) 
  {
    addRequirements(claw);
    this.mClaw = claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    this.mClaw.in();
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
    this.mClaw.rest();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
