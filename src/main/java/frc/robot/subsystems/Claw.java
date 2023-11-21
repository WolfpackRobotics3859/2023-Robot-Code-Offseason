// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  private static TalonFX mMotor;
  private static boolean mEngaged;

  public Claw() {
    mMotor = new TalonFX(Constants.CLAW.MOTOR_ID);
    mMotor.configFactoryDefault();
    mMotor.setNeutralMode(NeutralMode.Brake);
    mMotor.setSelectedSensorPosition(0);

    mMotor.configMotionCruiseVelocity(14000, 30);
    mMotor.configMotionAcceleration(14000, 30);
    mMotor.configMotionSCurveStrength(2, 30);

    mMotor.config_kP(0, 1);
    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

    mMotor.setInverted(true);
  }

  public void out() 
  {
    mMotor.set(ControlMode.MotionMagic, 0);
    mEngaged = false;
  }

  public void in() 
  {
    mMotor.set(ControlMode.PercentOutput, -0.15); //club rush changed from -0.15 
    mEngaged = true;
  }

  public void rest() 
  {
    mMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getEngaged() 
  {
    return mEngaged;
  }

  public void setEngaged(boolean engaged) 
  {
    mEngaged = engaged;
  }

  public void zeroMotorEncoder()
  {
    mMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Claw - Encoder Value", mMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Claw - isEngaged", getEngaged());
  }
}
