// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class Intake extends CommandBase {
  private Arm armSubsystem;
  /** Creates a new Intake. */
  public Intake(Arm armSubsystem) {
    this.armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.enableCoasting();
    armSubsystem.goToMMPosition(800);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.disableMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
