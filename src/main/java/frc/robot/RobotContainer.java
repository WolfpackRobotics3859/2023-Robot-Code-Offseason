// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.FirePositionOne;
import frc.robot.commands.arm.FirePositionTwo;
import frc.robot.commands.arm.Stow;
import frc.robot.subsystems.Arm;

public class RobotContainer 
{
  private final Arm mArm = new Arm();

  private final CommandXboxController mDriverController = new CommandXboxController(0);

  public RobotContainer() 
  {
    this.configureBindings();
  }

  private void configureBindings() 
  {
    mDriverController.a().onTrue(new FirePositionOne(mArm).andThen(new Stow(mArm)));
    mDriverController.b().onTrue(new FirePositionTwo(mArm).andThen(new Stow(mArm)));
  }

  public Command getAutonomousCommand() 
  {
    return Commands.print("No autonomous command configured");
  }
}
