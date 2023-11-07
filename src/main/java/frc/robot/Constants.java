package frc.robot;

/**
 * @brief Stores all robot specific constants.
 */
public final class Constants
{
    /**
     * @brief Contains constants related to hardware.
     */
    public static class Hardware
    {

        /**
         * @brief The CAN ID of swerve module 1's rotation motor.
         */
        public static final int MOTOR_SWERVE_1_ROTATION_ID = 2;

        /**
         * @brief The CAN ID of swerve module 1's thrust motor.
         */
        public static final int MOTOR_SWERVE_1_THRUST_ID = 1;

        /**
         * @brief The CAN ID of swerve module 1's encoder.
         */
        public static final int ENCODER_SWERVE_1 = 1;

        /**
         * @brief The CAN ID of swerve module 2's rotation motor.
         */
        public static final int MOTOR_SWERVE_2_ROTATION_ID = 4;

        /**
         * @brief The CAN ID of swerve module 2's thrust motor.
         */
        public static final int MOTOR_SWERVE_2_THRUST_ID = 3;

        /**
         * @brief The CAN ID of swerve module 2's encoder.
         */
        public static final int ENCODER_SWERVE_2 = 2;

        /**
         * @brief The CAN ID of swerve module 3's rotation motor.
         */
        public static final int MOTOR_SWERVE_3_ROTATION_ID = 6;

        /**
         * @brief The CAN ID of swerve module 3's thrust motor.
         */
        public static final int MOTOR_SWERVE_3_THRUST_ID = 5;

        /**
         * @brief The CAN ID of swerve module 3's encoder.
         */
        public static final int ENCODER_SWERVE_3 = 3;

        /**
         * @brief The CAN ID of swerve module 4's rotation motor.
         */
        public static final int MOTOR_SWERVE_4_ROTATION_ID = 0;

        /**
         * @brief The CAN ID of swerve module 4's thrust motor.
         */
        public static final int MOTOR_SWERVE_4_THRUST_ID = 0;

        /**
         * @brief The CAN ID of swerve module 4's encoder.
         */
        public static final int ENCODER_SWERVE_4 = 0;

        /**
         * @brief The CAN ID of the arm's primary motor.
         */
        public static final int MOTOR_ARM_1 = 12;

        /**
         * @brief The CAN ID of the arm's secondary motor.
         */
        public static final int MOTOR_ARM_2 = 0;

        /**
         * @brief The CAN ID of the arm's encoder.
         */
        public static final int ENCODER_ARM = 3;

        /**
         * @brief The CAN ID of the claw's motor.
         */
        public static final int MOTOR_CLAW = 0;

        /**
         * @brief The CAN ID of the central IMU.
         */
        public static final int IMU = 0;

        public static Throw SHORT_SHOT = new Throw();
       
    } // END CLASS HARDWARE

} // END CLASS CONSTANTS
