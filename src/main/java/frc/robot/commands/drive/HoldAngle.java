// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class HoldAngle extends CommandBase {
  private final Drivetrain drive;
  private final PIDController angController;

  private double omega;
  private double angle;

  /** Creates a new TurnToAngle. */
  public HoldAngle(Drivetrain drive, double angle) {
    this.drive = drive;
    this.angle = angle;
    angController = new PIDController(10, 0, 0.1);

    angController.enableContinuousInput(0, 360.0);
    
    angController.setTolerance(1);
 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angController.setSetpoint(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Jason - Yaw Value", drive.getYaw2());
    omega = angController.calculate(drive.getYaw2());
    SmartDashboard.putNumber("Jason - Omega", omega);
    drive.drive(new Translation2d(), omega, true, false);
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
