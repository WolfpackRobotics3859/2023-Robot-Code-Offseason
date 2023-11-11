// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.FireLongCone;
import frc.robot.commands.drive.Balance;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.drive.ResetGyro;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThrowTaxiBalance extends SequentialCommandGroup {
  /** Creates a new ThrowTaxiBalance. */
  public ThrowTaxiBalance(Drivetrain drivetrain, Arm arm, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetGyro(drivetrain).withTimeout(0.001),
     arm.zeroSensor(),
     new InstantCommand(() -> {claw.setEngaged(false);}), new FireLongCone(arm),
     new DriveBack(drivetrain, true, -13.0).withTimeout(4.5),
     new DriveBack(drivetrain, false, 0.0, true).withTimeout(0.75),
     new DriveBack(drivetrain, true, 13.0).withTimeout(3.5), 
     new Balance(drivetrain).repeatedly()
    );
  }
}
