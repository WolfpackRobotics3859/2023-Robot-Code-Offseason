// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private TalonFX mMotor;
  /** Creates a new Climb. */
  public Climb() {
    mMotor = new TalonFX(Constants.CLIMB.MOTOR_ID);
    mMotor.configFactoryDefault();
  }

  public void setPercent(double percent) {
    mMotor.set(ControlMode.PercentOutput, percent);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
