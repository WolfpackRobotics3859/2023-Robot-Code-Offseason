// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.HoldAngle;
import frc.robot.commands.drive.ResetGyro;
import frc.robot.commands.drive.StopRobot;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.arm.FireLow;
import frc.robot.commands.arm.Intake;
import frc.robot.commands.arm.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.commands.claw.Close;
import frc.robot.commands.claw.Open;
import frc.robot.subsystems.Claw;
// import frc.robot.subsystems.Climb;
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
        //Auto Chooser
    NamedCommands.registerCommand("stow", new Stow(mArm));
    NamedCommands.registerCommand("intake", new Intake(mArm));
    NamedCommands.registerCommand("fire", new FireLow(mArm));
    NamedCommands.registerCommand("open",  new Open(mClaw));
    NamedCommands.registerCommand("close", new Close(mClaw));
    NamedCommands.registerCommand("stop", new StopRobot(mDrive));

    mChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", mChooser);
    //Regular Driving
     mDrive.setDefaultCommand(
      new Drive(
          mDrive, 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.TRANSLATION_AXIS), 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.STRAFE_AXIS), 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.ROTATION_AXIS)*-1, 
          () -> robotCentric.getAsBoolean()
        )
    );

    mDriverController.rightTrigger(0.1).whileTrue(
      new Drive(
          mDrive, 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.TRANSLATION_AXIS)*SwerveConstants.DRIVE_SPEED, 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.STRAFE_AXIS)*SwerveConstants.DRIVE_SPEED, 
          () -> mDriverController.getRawAxis(XboxController.Axis.kRightTrigger.value)*-0.1,
          () -> robotCentric.getAsBoolean()
        )
    );

    mDriverController.leftTrigger(0.1).whileTrue(
      new Drive(
          mDrive, 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.TRANSLATION_AXIS)*SwerveConstants.DRIVE_SPEED, 
          () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.STRAFE_AXIS)*SwerveConstants.DRIVE_SPEED, 
          () -> mDriverController.getRawAxis(XboxController.Axis.kLeftTrigger.value)*0.1,
          () -> robotCentric.getAsBoolean()
        )
    );

    //Slow Mode
    mDriverController.rightBumper().whileTrue(
      new Drive(
        mDrive, 
        () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.TRANSLATION_AXIS)*SwerveConstants.SLOW_DRIVE_SPEED, 
        () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.STRAFE_AXIS)*SwerveConstants.SLOW_DRIVE_SPEED, 
        () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.ROTATION_AXIS)*-SwerveConstants.SLOW_DRIVE_SPEED, 
        () -> false
      )
    );
    
    configureBindings();
  }

  private void configureBindings() 
  {
    // Fine Positioning
    mOperatorController.povLeft().or(mOperatorController.povDownLeft()).or(mOperatorController.povUpLeft()).whileTrue(new HoldAngle(mDrive, 0, () -> 0, () -> Constants.SwerveConstants.FINE_DRIVE_SPEED));
    mOperatorController.povRight().or(mOperatorController.povDownRight()).or(mOperatorController.povUpRight()).whileTrue(new HoldAngle(mDrive, 0, () -> 0, () -> -Constants.SwerveConstants.FINE_DRIVE_SPEED));
    mOperatorController.povUp().whileTrue(new HoldAngle(mDrive, 0, () -> Constants.SwerveConstants.FINE_DRIVE_SPEED, () -> 0));
    mOperatorController.povDown().whileTrue(new HoldAngle(mDrive, 0, () -> -Constants.SwerveConstants.FINE_DRIVE_SPEED, () -> 0));
    
    // // Claw Toggle
    // mOperatorController.rightBumper().onTrue(new InstantCommand(() -> {mClaw.setEngaged(!mClaw.getEngaged());}));

    // // Arm Control
    // mOperatorController.leftTrigger(0.1).onTrue(new SequentialCommandGroup(new InstantCommand(() -> {mClaw.setEngaged(false);}), new WaitCommand(0.1), new FireShortCone(mArm).andThen(new Stow(mArm))));
    // mOperatorController.rightTrigger(0.1).onTrue(new SequentialCommandGroup(new InstantCommand(() -> {mClaw.setEngaged(false);}), new WaitCommand(0.1), new FireLongCone(mArm).andThen(new Stow(mArm))));
    // mOperatorController.y().onTrue(new SequentialCommandGroup(new InstantCommand(() -> {mClaw.setEngaged(false);}), new WaitCommand(0.1), new FireLongCube(mArm).andThen(new Stow(mArm))));
    // mOperatorController.b().onTrue(new SequentialCommandGroup(new InstantCommand(() -> {mClaw.setEngaged(false);}), new WaitCommand(0.1), new FireLow(mArm).andThen(new Stow(mArm))));
    
    // //Intake
    // mOperatorController.x().whileTrue(new Intake(mArm));

    //Reset Gyro
    mDriverController.a().onTrue(new ResetGyro(mDrive));

    //Turn to angle
    mOperatorController.leftBumper().whileTrue(new TurnToAngle(mDrive, 0));
    SmartDashboard.putData(new TurnToAngle(mDrive, 0));
    
    //Squaring
    mDriverController.b().whileTrue(new HoldAngle(mDrive, 90,
    () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.TRANSLATION_AXIS)*SwerveConstants.DRIVE_SPEED,
    () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.STRAFE_AXIS)*SwerveConstants.DRIVE_SPEED));
    //Squaring
    mDriverController.x().whileTrue(new HoldAngle(mDrive, 270,
    () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.TRANSLATION_AXIS)*SwerveConstants.DRIVE_SPEED,
    () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.STRAFE_AXIS)*SwerveConstants.DRIVE_SPEED));
  
    mDriverController.y().whileTrue(new HoldAngle(mDrive, 0,
    () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.TRANSLATION_AXIS)*SwerveConstants.DRIVE_SPEED,
    () -> mDriverController.getRawAxis(Constants.CONTROLLERS.DRIVER_AXES.STRAFE_AXIS)*SwerveConstants.DRIVE_SPEED));    


    SmartDashboard.putData("Intake Command", new Intake(mArm));
    SmartDashboard.putData("Stow", new Stow(mArm));
  }

  public Command getAutonomousCommand() 
  {
    return mChooser.getSelected();
  }
}
