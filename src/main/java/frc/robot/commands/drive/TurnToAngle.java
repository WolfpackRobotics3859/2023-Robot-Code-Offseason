// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {
  private final Drivetrain drive;
  private Pose2d target;
  private Supplier<Pose2d> targetSupplier;
  private final PIDController angController;

  private Pose2d currentPose;
  private double omega;
  private int angle;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(Drivetrain drive, int angle) {
    this.drive = drive;
    this.angle = angle;
    angController = new PIDController(10, 0, 0.1);

    angController.enableContinuousInput(-Math.PI, Math.PI);
    
    angController.setTolerance(0.01);
 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angController.setSetpoint(Rotation2d.fromDegrees(angle).getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = drive.getPose();
    omega = angController.calculate(drive.getYaw().getRadians());
    drive.drive(new Translation2d(0, 0), omega, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new Translation2d(), 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (angController.atSetpoint());
  }
}
