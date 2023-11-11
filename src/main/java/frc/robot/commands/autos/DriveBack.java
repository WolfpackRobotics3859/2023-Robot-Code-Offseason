// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveBack extends CommandBase {
  private Drivetrain mDrivetrain;
  private boolean mTransition;
  private double mEndpoint;
  private boolean mReversed;

  /** Creates a new DriveBack. */
  public DriveBack(Drivetrain drivetrain, boolean transition, double endpoint) {
    mDrivetrain = drivetrain;
    mTransition = transition;
    mEndpoint = endpoint;
    mReversed = false;
    addRequirements(drivetrain);
  }

  public DriveBack(Drivetrain drivetrain, boolean transition, double endpoint, boolean reversed) {
    mDrivetrain = drivetrain;
    mTransition = transition;
    mEndpoint = endpoint;
    mReversed  = reversed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mReversed) {
      mDrivetrain.drive(new Translation2d(0.25, 0).times(Constants.SwerveConstants.maxSpeed),
               0, false, true);
    } else {
      mDrivetrain.drive(new Translation2d(-0.25, 0).times(Constants.SwerveConstants.maxSpeed),
               0, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(mTransition){
      return (Math.abs(mDrivetrain.gyro.getRoll()) > mEndpoint);
    } else {
      return false;
    }
  }
}
