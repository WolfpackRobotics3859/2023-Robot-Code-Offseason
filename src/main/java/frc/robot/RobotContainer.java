// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.arm.FireLongCone;
import frc.robot.commands.arm.FireShortCone;
import frc.robot.commands.arm.Stow;
import frc.robot.subsystems.Arm;

public class RobotContainer 
{
  private final Arm mArm = new Arm();

  private final CommandXboxController mDriverController = new CommandXboxController(0);
  public final static CommandXboxController secondaryController = new CommandXboxController(1);
  
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final JoystickButton robotCentric = new JoystickButton(mDriverController.getHID(), XboxController.Button.kRightBumper.value);

  public RobotContainer() 
  { 
     driveSubsystem.setDefaultCommand(
      new DriveCommand(
          driveSubsystem, 
          () -> mDriverController.getRawAxis(translationAxis)*0.80, 
          () -> mDriverController.getRawAxis(strafeAxis)*0.80, 
          () -> -mDriverController.getRawAxis(rotationAxis)*0.80, 
          () -> robotCentric.getAsBoolean()
        )
    );
    configureBindings();
  }

  private void configureBindings() 
  {
    secondaryController.a().onTrue(new FireShortCone(mArm).andThen(new Stow(mArm)));
    secondaryController.b().onTrue(new FireLongCone(mArm).andThen(new Stow(mArm)));
    secondaryController.y().onTrue(mArm.zeroSensor());
    secondaryController.rightBumper().onTrue(mArm.playTheMusic());
    secondaryController.povLeft().or(secondaryController.povDownLeft()).or(secondaryController.povUpLeft()).whileTrue(new DriveCommand(driveSubsystem, () -> 0.0, () ->0.2, () -> 0, () -> true));
    secondaryController.povRight().or(secondaryController.povDownRight()).or(secondaryController.povUpRight()).whileTrue(new DriveCommand(driveSubsystem, () -> 0, () ->-0.2, () -> 0, () -> true));
    secondaryController.povUp().whileTrue(new DriveCommand(driveSubsystem, () -> 0.2, () ->0, () -> 0, () -> true));
    secondaryController.povDown().whileTrue(new DriveCommand(driveSubsystem, () -> -0.2, () ->0, () -> 0, () -> true));
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}
