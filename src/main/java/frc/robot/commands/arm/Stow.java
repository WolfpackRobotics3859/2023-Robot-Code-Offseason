// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Stow extends Command
{
  private Arm mArm;
  /** Creates a new Stow. */
  public Stow(Arm arm) 
  {
    addRequirements(arm);
    mArm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    this.mArm.setPercent(-0.4);
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
    this.mArm.disableMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return this.mArm.reachedLowerSoftStop();
  }
}
