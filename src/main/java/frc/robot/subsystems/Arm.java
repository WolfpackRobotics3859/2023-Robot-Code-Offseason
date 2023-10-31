// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @brief Arm subsystem for the 2023 Robot.
 */
public class Arm extends SubsystemBase 
{
  // Add these numbers to constants once the constants issue is completed.
  private static TalonFX mTalon1 = new TalonFX(12);
  private static CANCoder mEncoder1 = new CANCoder(5);
  private static TalonFXConfiguration mTalon1Config = new TalonFXConfiguration();

  /**
   * @brief Creates the Arm subsystem.
   */
  public Arm()
  {
    mTalon1.configFactoryDefault();
    // mTalon1Config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
    mTalon1.configRemoteFeedbackFilter(mEncoder1, 0);
    mTalon1.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    mTalon1.setSelectedSensorPosition(0);
    // mTalon1.configAllSettings(mTalon1Config);
    mTalon1.setSensorPhase(true);
    mTalon1.setNeutralMode(NeutralMode.Coast);
    mTalon1.setInverted(true);
    mEncoder1.configSensorDirection(true);
    mTalon1.configVoltageCompSaturation(8);
    mTalon1.enableVoltageCompensation(true);
  }

  public CommandBase setMotorToCoast()
  {
    return runOnce(
      () -> {
        mTalon1.setNeutralMode(NeutralMode.Coast);
      });
  }

  public void setPercent(double percentage)
  {
    mTalon1.set(ControlMode.PercentOutput, percentage);
  }

  public void enableCoasting()
  {
    mTalon1.setNeutralMode(NeutralMode.Coast);
  }

  public void enableBraking()
  {
    mTalon1.setNeutralMode(NeutralMode.Brake);
  }

  public void disableMotor()
  {
    this.enableCoasting();
    this.setPercent(0);
  }

  public boolean reachedPositionOne()
  {
    return mTalon1.getSelectedSensorPosition() > 0;
  }

  public boolean reachedPositionTwo()
  {
    return mTalon1.getSelectedSensorPosition() > 300;
  }

  public boolean reachedLowerSoftStop()
  {
    return mTalon1.getSelectedSensorPosition() < 100;
  }

  /**
   * @brief This method runs once per scheduler run.
   */
  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Thrower Motor Units", mTalon1.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Reached Position One", this.reachedPositionOne());
    SmartDashboard.putBoolean("Reached Position Two", this.reachedPositionTwo());
    SmartDashboard.putBoolean("Reached Lower Soft Stop", this.reachedLowerSoftStop());
  }
}
