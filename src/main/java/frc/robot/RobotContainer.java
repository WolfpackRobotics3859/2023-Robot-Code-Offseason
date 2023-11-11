// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.arm.FireLongCone;
import frc.robot.commands.arm.FireShortCone;
import frc.robot.commands.arm.Stow;
import frc.robot.commands.autos.Throw;
import frc.robot.commands.autos.ThrowTaxi;
import frc.robot.commands.autos.ThrowTaxiBalance;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.claw.Close;
import frc.robot.commands.claw.Open;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer 
{
  // Controller Declarations
  private static CommandXboxController mDriverController = new CommandXboxController(Constants.CONTROLLERS.DRIVER_CONTROLLER_ID);
  private static CommandXboxController mOperatorController = new CommandXboxController(Constants.CONTROLLERS.OPERATOR_CONTROLLER_ID);

  // Subsystem Declarations
  private final Arm mArm = new Arm();
  private final Drivetrain mDrive = new Drivetrain();
  private final Claw mClaw = new Claw();
  
  //Auto Sendable Chooser
  SendableChooser<Command> mChooser = new SendableChooser<Command>();
  // Please check if code below can remove this object declaration
  private final JoystickButton robotCentric = new JoystickButton(mDriverController.getHID(), XboxController.Button.kRightBumper.value);

  public RobotContainer() 
  {
    // Hard coding drive speed limits might be able to be done within swerve constants instead of scaling the axes. Please check.
     mDrive.setDefaultCommand(
      new Drive(
          mDrive, 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.TRANSLATION_AXIS)*0.80, 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.STRAFE_AXIS)*0.80, 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.ROTATION_AXIS)*0.80, 
          () -> robotCentric.getAsBoolean()
        )
    );

    // Please Check if this works instead of creating an extra variable
    // mDrive.setDefaultCommand(
    //   new Drive(
    //       mDrive, 
    //       () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.TRANSLATION_AXIS)*0.80, 
    //       () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.STRAFE_AXIS)*0.80, 
    //       () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.ROTATION_AXIS)*0.80, 
    //       () -> mDriverController.rightBumper().getAsBoolean()
    //     )
    // );

    // Logic for this should be done inside the subsystem instead.
    Trigger armsEngagedTrigger = new Trigger(mClaw::getEngaged);
    armsEngagedTrigger.whileFalse(new Open(mClaw));
    armsEngagedTrigger.whileTrue(new Close(mClaw));
    
    SmartDashboard.putData(mClaw);
    SmartDashboard.putData(new Close(mClaw));
    SmartDashboard.putData(new Open(mClaw));
    
    //Auto Chooser
    mChooser.setDefaultOption("ThrowTaxiBalance", new ThrowTaxiBalance(mDrive, mArm, mClaw));
    mChooser.addOption("ThrowTaxi", new ThrowTaxi(mDrive, mArm, mClaw));
    mChooser.addOption("Throw", new Throw(mDrive, mArm, mClaw));
    
    configureBindings();
  }

  private void configureBindings() 
  {
    // Fine Positioning
    mOperatorController.povLeft().or(mOperatorController.povDownLeft()).or(mOperatorController.povUpLeft()).whileTrue(new Drive(mDrive, () -> 0.0, () ->0.1, () -> 0, () -> true));
    mOperatorController.povRight().or(mOperatorController.povDownRight()).or(mOperatorController.povUpRight()).whileTrue(new Drive(mDrive, () -> 0, () ->-0.1, () -> 0, () -> true));
    mOperatorController.povUp().whileTrue(new Drive(mDrive, () -> 0.1, () ->0, () -> 0, () -> true));
    mOperatorController.povDown().whileTrue(new Drive(mDrive, () -> -0.1, () ->0, () -> 0, () -> true));

    // Claw Toggle
    mOperatorController.rightBumper().onTrue(new InstantCommand(() -> {mClaw.setEngaged(!mClaw.getEngaged());}));

    // Arm Control
    mOperatorController.a().onTrue(new FireShortCone(mArm).andThen(new Stow(mArm)));
    mOperatorController.b().onTrue(new FireLongCone(mArm).andThen(new Stow(mArm)));
    mOperatorController.y().onTrue(mArm.zeroSensor());
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}
