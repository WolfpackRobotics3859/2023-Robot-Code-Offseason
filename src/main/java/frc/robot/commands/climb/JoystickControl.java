// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class JoystickControl extends Command {
  private DoubleSupplier mPercentSupplier;
  private Climb mClimb;
  /** Creates a new JoystickControl. */
  public JoystickControl(Climb climb, DoubleSupplier supplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    mPercentSupplier = supplier;
    mClimb = climb;
    addRequirements(mClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimb.setPercent(mPercentSupplier.getAsDouble());
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
