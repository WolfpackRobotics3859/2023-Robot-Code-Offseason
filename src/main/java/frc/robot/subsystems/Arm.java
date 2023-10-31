// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.music.Orchestra;
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
  private static TalonFX mTalon2 = new TalonFX(13);
  private static CANCoder mEncoder1 = new CANCoder(5);
  private static TalonFXConfiguration mTalon1Config = new TalonFXConfiguration();
  private static Orchestra musica;

  private ArrayList<TalonFX> instruments;

  String theSong = "theSong.chrp";

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

    SmartDashboard.setDefaultNumber("Arm Motion 1 Speed", 0.92);
    SmartDashboard.setDefaultNumber("Arm Motion 2 Segment 1 Speed", 0);
    SmartDashboard.setDefaultNumber("Arm Motion 2 Segment 2 Speed", 0);
    SmartDashboard.setDefaultNumber("Arm Motion 1 Ending Position", 300);
    SmartDashboard.setDefaultNumber("Arm Motion 2 Segment 1 Ending Position", 0);
    SmartDashboard.setDefaultNumber("Arm Motion 2 Segment 2 Ending Position", 0);

    instruments.add(mTalon1);
    instruments.add(mTalon2);
    musica = new Orchestra(instruments);
    musica.loadMusic(theSong);
  }

  public CommandBase zeroSensor()
  {
    return runOnce(
      () -> {
        mEncoder1.setPosition(0);
      });
  }

  public CommandBase playTheMusic()
  {
    return runOnce(
      () -> {
        musica.play();
      });
  }

  public void setPercent(double percentage)
  {
    mTalon1.set(ControlMode.PercentOutput, percentage);
  }

  public boolean runSingleSegmentThrow(double power, double endingPosition)
  {
    if(reachedGoalPosition(endingPosition))
    {
      return true;
    }
    mTalon1.set(ControlMode.PercentOutput, power);
    return false;
  }

  public boolean runDoubleSegmentThrow(double segment1Speed, double segment1EndingPosition, double segment2Speed, double segment2EndingPosition)
  {
    if(reachedGoalPosition(segment2EndingPosition))
    {
      return true;
    }
    if(reachedGoalPosition(segment1EndingPosition))
    {
      mTalon1.set(ControlMode.PercentOutput, segment2Speed);
    }
    else
    {
      mTalon1.set(ControlMode.PercentOutput, segment1Speed);
    }
    return false;
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

  public boolean reachedGoalPosition(double position)
  {
    return mTalon1.getSelectedSensorPosition() > position;
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
  }
}
