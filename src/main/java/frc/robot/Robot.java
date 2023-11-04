// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.CTREConfigs;

/**
 * @brief Robot class that exposes processing.
 */
public class Robot extends TimedRobot 
{
  private Command m_autonomousCommand;
  public static CTREConfigs ctreConfigs;
  private RobotContainer m_robotContainer;

  /**
   * @brief Runs once on boot-up or code reset.
   */
  @Override
  public void robotInit() 
  {
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();
    RoboLogger.init();
  }

  /**
   * @brief Loops no matter what state the robot is in.
   */
  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  /**
   * @brief Runs once upon robot disabling.
   */
  @Override
  public void disabledInit()
  {
    // Intentionally Empty
  }

  /**
   * @brief Loops while the robot is disabled.
   */
  @Override
  public void disabledPeriodic() 
  {
    // Intentionally Empty
  }

  /**
   * @brief Runs once upon exit from disabled.
   */
  @Override
  public void disabledExit()
  {
    // Intentionally Empty
  }

  /**
   * @brief Runs once when autonomous begins.
   */
  @Override
  public void autonomousInit() 
  {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * @brief Loops during autonmous period.
   */
  @Override
  public void autonomousPeriodic()
  {
    // Intentionally Empty
  }

  /**
   * @brief Runs once upon exit from autonomous.
   */
  @Override
  public void autonomousExit() 
  {
    // Intentionally Empty
  }

  /**
   * @brief Runs once at the beginning of teleop.
   */
  @Override
  public void teleopInit() 
  {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * @brief Loops during teleop period.
   */
  @Override
  public void teleopPeriodic()
  {
    // Intentionally Empty
  }

  /**
   * @brief Runs once upon exit from teleop.
   */
  @Override
  public void teleopExit()
  {
    // Intentionally Empty
  }

  /**
   * @brief Runs once when testing starts.
   */
  @Override
  public void testInit()
  {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * @brief Loops during testing.
   */
  @Override
  public void testPeriodic()
  {
    // Intentionally Empty
  }

  /**
   * @brief Runs once upon exit from testing.
   */
  @Override
  public void testExit() 
  {
    // Intentionally Empty
  }

} // END CLASS ROBOT
