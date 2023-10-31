// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class FirePositionTwo extends CommandBase {
  private Arm mArm;

  /** Creates a new FirePositionOne. */
  public FirePositionTwo(Arm arm) {
    addRequirements(arm);
    mArm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    mArm.setPercent(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mArm.reachedPositionTwo();
  }
}
