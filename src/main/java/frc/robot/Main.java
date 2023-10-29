// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * @brief Top level class for the robot program.
 */
public final class Main {

  /**
   * @brief Private constructor for main.
   */
  private Main() 
  {
    // Intentionally Empty
  } // END CONSTRUCTOR MAIN

  /**
   * @brief Program entry point.
   * @param args Arguments for the program.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  } // END METHOD MAIN
} // END CLASS MAIN
