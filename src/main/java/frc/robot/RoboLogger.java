package frc.robot;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class RoboLogger 
{
   // BEGIN CUSTOM LOG ENTRIES
   
   /**
    * @brief An example custom boolean log entry.
    */
   public static BooleanLogEntry mBooleanLogExample;

   /**
    * @brief An example custom double log entry.
    */
   public static DoubleLogEntry mDoubleLogExample;

   /**
    * @brief An example custom string log entry.
    */
   public static StringLogEntry mStringLogExample;

   // END CUSTOM LOG ENTRIES

   /**
    * @brief Initializes the RoboLogger and custom log fields.
    */
   public static void init()
   {

      // Begins recording any NetworkTables changes to log.
      DataLogManager.start();

      // Begins recording any Driver Station or joystick to log.
      DriverStation.startDataLog(DataLogManager.getLog());

      // Alternative Driver Station log begin without joystick.
      // DriverStation.startDataLog(DataLogManager.getLog());

      DataLog log = DataLogManager.getLog();
      
      /* Now we must initialize our logger variables. To log values to these
       * fields all you have to do is key.append(value) in robot code. For 
       * example mBooleanLogExample.append(true);
       */
      
      mBooleanLogExample = new BooleanLogEntry(log, "example/Boolean");

      mDoubleLogExample = new DoubleLogEntry(log, "example/Double");

      mStringLogExample = new StringLogEntry(log, "example/String");

   } // END METHOD INIT()
} // END CLASS ROBOLOGGER
