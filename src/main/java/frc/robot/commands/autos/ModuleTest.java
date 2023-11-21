// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ModuleTest extends CommandBase {
  private Drivetrain mDrivetrain;
  /** Creates a new ModuleTest. */
  public ModuleTest(Drivetrain drivetrain) {
    mDrivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveModuleState[] states = {new SwerveModuleState(1, new Rotation2d()),
                                    new SwerveModuleState(1, new Rotation2d()),
                                    new SwerveModuleState(1, new Rotation2d()),
                                    new SwerveModuleState(1, new Rotation2d())};
    mDrivetrain.setModuleStates(states, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stop motion
    mDrivetrain.drive(new Translation2d(), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
