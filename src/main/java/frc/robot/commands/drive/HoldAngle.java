// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drivetrain;

public class HoldAngle extends CommandBase {
  private final Drivetrain drive;
  private final PIDController angController;

  private double omega;
  private double angle;
  private DoubleSupplier mTranslation;
  private DoubleSupplier mStrafe;

  /** Creates a new TurnToAngle. */
  public HoldAngle(Drivetrain drive, double angle, DoubleSupplier translation, DoubleSupplier strafe) {
    this.drive = drive;
    this.angle = angle;
    angController = new PIDController(10, 0, 0.1);

    angController.enableContinuousInput(-Math.PI, Math.PI);
    
    angController.setTolerance(0.01);

    mTranslation = translation;
    mStrafe = strafe;
 
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
    SmartDashboard.putNumber("Jason - Yaw Value", drive.getYaw2());
    omega = angController.calculate(drive.getYaw().getRadians());
    SmartDashboard.putNumber("Jason - Omega", omega);
    double translationVal = MathUtil.applyDeadband(mTranslation.getAsDouble(), 0.03);
    double strafeVal = MathUtil.applyDeadband(mStrafe.getAsDouble(), 0.03);
    drive.drive(new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), omega, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new Translation2d(), 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
