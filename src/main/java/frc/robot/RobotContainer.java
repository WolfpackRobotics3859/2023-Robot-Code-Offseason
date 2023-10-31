// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.FireLongCone;
import frc.robot.commands.arm.FireShortCone;
import frc.robot.commands.arm.Stow;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.claw.ClawCloseCommand;
import frc.robot.commands.claw.ClawOpenCommand;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer 
{
  private final CommandXboxController mDriverController = new CommandXboxController(0);
  public final static CommandXboxController secondaryController = new CommandXboxController(1);

  private final Arm mArm = new Arm();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
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
          () -> mDriverController.getRawAxis(rotationAxis)*0.80, 
          () -> robotCentric.getAsBoolean()
        )
    );

    Trigger armsEngagedTrigger = new Trigger(clawSubsystem::isClawEngaged);
    armsEngagedTrigger.whileFalse(new ClawOpenCommand(clawSubsystem));
    armsEngagedTrigger.whileTrue(new ClawCloseCommand(clawSubsystem));
    
    SmartDashboard.putData(clawSubsystem);
    SmartDashboard.putData(new ClawCloseCommand(clawSubsystem));
    SmartDashboard.putData(new ClawOpenCommand(clawSubsystem));

    secondaryController.povLeft().or(secondaryController.povDownLeft()).or(secondaryController.povUpLeft()).whileTrue(new DriveCommand(driveSubsystem, () -> 0.0, () ->0.1, () -> 0, () -> true));
    secondaryController.povRight().or(secondaryController.povDownRight()).or(secondaryController.povUpRight()).whileTrue(new DriveCommand(driveSubsystem, () -> 0, () ->-0.1, () -> 0, () -> true));
    secondaryController.povUp().whileTrue(new DriveCommand(driveSubsystem, () -> 0.1, () ->0, () -> 0, () -> true));
    secondaryController.povDown().whileTrue(new DriveCommand(driveSubsystem, () -> -0.1, () ->0, () -> 0, () -> true));
    
    configureBindings();

  }

  private void configureBindings() 
  {
    secondaryController.povLeft().or(secondaryController.povDownLeft()).or(secondaryController.povUpLeft()).whileTrue(new DriveCommand(driveSubsystem, () -> 0.0, () ->0.1, () -> 0, () -> true));
    secondaryController.povRight().or(secondaryController.povDownRight()).or(secondaryController.povUpRight()).whileTrue(new DriveCommand(driveSubsystem, () -> 0, () ->-0.1, () -> 0, () -> true));
    secondaryController.povUp().whileTrue(new DriveCommand(driveSubsystem, () -> 0.1, () ->0, () -> 0, () -> true));
    secondaryController.povDown().whileTrue(new DriveCommand(driveSubsystem, () -> -0.1, () ->0, () -> 0, () -> true));

    secondaryController.rightBumper().onTrue(new InstantCommand(() -> {clawSubsystem.setArmState(!clawSubsystem.isClawEngaged());}));

    mDriverController.a().onTrue(new FireShortCone(mArm).andThen(new Stow(mArm)));
    mDriverController.b().onTrue(new FireLongCone(mArm).andThen(new Stow(mArm)));
    mDriverController.y().onTrue(mArm.zeroSensor());
    mDriverController.rightBumper().onTrue(mArm.playTheMusic());
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}
