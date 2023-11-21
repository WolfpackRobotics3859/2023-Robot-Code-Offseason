// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Balance extends Command {
  /** Creates a new Balance. */
  Drivetrain mDrivetrain;
  public Balance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivetrain = drivetrain;
    addRequirements(mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = mDrivetrain.gyro.getRoll();
    double output = -0.175 * (angle/Math.abs(angle));

    if(Math.abs(angle) > 11.0){
      mDrivetrain.drive(new Translation2d(output, 0).times(Constants.SwerveConstants.maxSpeed), 0, true, true);
    } else {
      mDrivetrain.drive(new Translation2d(0, 0), 0, true, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
