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

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  private TalonFX clawMotor;
  private boolean armsEngaged;

  public ClawSubsystem() {
    clawMotor = new TalonFX(Constants.ClawConstants.clawMotorId);
    clawMotor.configFactoryDefault();
    clawMotor.setNeutralMode(NeutralMode.Brake);
    clawMotor.setSelectedSensorPosition(0);

    clawMotor.configMotionCruiseVelocity(14000, 30);
    clawMotor.configMotionAcceleration(14000, 30);
    clawMotor.configMotionSCurveStrength(2, 30);

    clawMotor.config_kP(0, 1);
    clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    clawMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

    clawMotor.setInverted(true);
  }

  public void armsOut() {
    clawMotor.set(ControlMode.MotionMagic, 0);
    armsEngaged = false;
  }

  public void armsIn() {
    clawMotor.set(ControlMode.PercentOutput, -0.15); //club rush changed from -0.15 
    armsEngaged = true;
  }

  public void rest() {
    clawMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isClawEngaged() {
    return armsEngaged;
  }

  public void setArmState(boolean engaged) {
    armsEngaged = engaged;
  }

  public void zero() {
    clawMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Chris Cross Encoder", clawMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("isEngaged", isClawEngaged());

  }
}
