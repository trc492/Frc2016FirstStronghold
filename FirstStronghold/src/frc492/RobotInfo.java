package frc492;

public class RobotInfo
{
    //
    // Compiler switches
    //
    public static final boolean ENABLE_LEDS             = true;
    
    //
    // Joystick ports.
    //
    public static final int JSPORT_LEFT_DRIVESTICK      = 0;
    public static final int JSPORT_RIGHT_DRIVESTICK     = 1;
    public static final int JSPORT_OPERATORSTICK        = 2;

    //
    // Digital Output ports.
    //
    public static final int DOUT_LEFTLIGHT_RED          = 0;
    public static final int DOUT_LEFTLIGHT_GREEN        = 1;
    public static final int DOUT_LEFTLIGHT_BLUE         = 2;
    public static final int DOUT_RIGHTLIGHT_RED         = 3;
    public static final int DOUT_RIGHTLIGHT_GREEN       = 4;
    public static final int DOUT_RIGHTLIGHT_BLUE        = 5;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTMOTOR        = 3;    //40A: Orange
    public static final int CANID_RIGHTFRONTMOTOR       = 4;    //40A: Yellow
    public static final int CANID_LEFTREARMOTOR         = 5;    //40A: Green
    public static final int CANID_RIGHTREARMOTOR        = 6;    //40A: Blue
    public static final int CANID_RESERVED1             = 7;    //30A: Purple
    public static final int CANID_RIGHT_ARM             = 8;    //30A: Gray
    public static final int CANID_CRANE                 = 9;    //40A: White
    public static final int CANID_TILTER                = 10;   //40A: Orange
    public static final int CANID_LEFT_ARM              = 11;   //40A: Yellow
    public static final int CANID_WINCH                 = 12;   //40A: Green
    public static final int CANID_PICKUP                = 13;   //30A: Blue
    public static final int CANID_RESERVED2             = 14;   //30A: Purple
    
    public static final int CANID_PDP                   = 16;
    public static final int CANID_PCM1                  = 17;
    public static final int CANID_PCM2                  = 18;

    //
    // Relay channels.
    //
    public static final int RELAY_RINGLIGHT_POWER       = 0;

    //
    // Solenoid channels.
    //
    public static final int SOL_LED_RED                 = 7;
    public static final int SOL_LED_GREEN               = 6;
    public static final int SOL_LED_BLUE                = 5;

    //
    // Miscellaneous sensors and devices.
    //
    public static final double ULTRASONIC_INCHESPERVOLT = (512.0/5.0);
    public static final String USB_CAM_NAME             = "cam0";

    //
    // DriveBase subsystem.
    //
    public static final double DRIVEBASE_X_SCALE        = 0.01321074310684615838703943056885;
    public static final double ENCODER_X_KP             = 0.045;
    public static final double ENCODER_X_KI             = 0.0;
    public static final double ENCODER_X_KD             = 0.0;
    public static final double ENCODER_X_KF             = 0.0;
    public static final double ENCODER_X_TOLERANCE      = 1.0;
    public static final double ENCODER_X_SETTLING       = 0.2;
    public static final double DRIVEBASE_Y_SCALE        = 0.01621544056690844936881352331858;
    public static final double ENCODER_Y_KP             = 0.025;//0.018;
    public static final double ENCODER_Y_KI             = 0.0;
    public static final double ENCODER_Y_KD             = 0.01;
    public static final double ENCODER_Y_KF             = 0.0;
    public static final double ENCODER_Y_TOLERANCE      = 1.0;
    public static final double ENCODER_Y_SETTLING       = 0.2;
    public static final double GYRO_TURN_KP             = 0.02;//0.010;
    public static final double GYRO_TURN_KI             = 0.0;
    public static final double GYRO_TURN_KD             = 0.0;
    public static final double GYRO_TURN_KF             = 0.0;
    public static final double GYRO_TURN_TOLERANCE      = 1.0;
    public static final double GYRO_TURN_SETTLING       = 0.2;
    public static final double SONAR_Y_KP               = 0.035;    //???
    public static final double SONAR_Y_KI               = 0.0;
    public static final double SONAR_Y_KD               = 0.0;
    public static final double SONAR_Y_KF               = 0.0;
    public static final double SONAR_Y_TOLERANCE        = 0.5;
    public static final double SONAR_Y_SETTLING         = 0.2;
    public static final double DRIVE_SLOW_XSCALE        = 3.0;
    public static final double DRIVE_SLOW_YSCALE        = 3.0;
    public static final double DRIVE_SLOW_TURNSCALE     = 3.0;

    //
    // Arm subsystem.
    //
    public static final double ARM_DEGREES_PER_COUNT    = (180.0/100.0);
    public static final double ARM_CAL_POWER            = 0.5;
    public static final double ARM_SYNC_GAIN            = 0.05;
    public static final double ARM_KP                   = 0.02;
    public static final double ARM_KI                   = 0.0;
    public static final double ARM_KD                   = 0.0;
    public static final double ARM_KF                   = 0.0;
    public static final double ARM_TOLERANCE            = 1.0;
    public static final double ARM_SETTLING             = 0.2;
    public static final double ARM_MIN_POSITION         = 0.0;
    public static final double ARM_MAX_POSITION         = 180.0;
    public static final double ARM_UP_POSITION          = ARM_MIN_POSITION;
    public static final double ARM_OUT_POSITION         = 90.0;
    public static final double ARM_GROUND_POSITION      = 120.0;
    public static final double ARM_DOWN_POSITION        = ARM_MAX_POSITION;
 
    //
    // Crane subsystem.
    //
    public static final double WINCH_INCHES_PER_COUNT   = (50.625/3683087.0);//(50.25/741.0);
    public static final double WINCH_POWER              = 1.0;
    public static final double WINCH_KP                 = 1.0;
    public static final double WINCH_KI                 = 0.0;
    public static final double WINCH_KD                 = 0.0;
    public static final double WINCH_KF                 = 0.0;
    public static final double WINCH_TOLERANCE          = 1.0;
    public static final double WINCH_SETTLING           = 0.2;
    public static final double WINCH_MAX_LENGTH         = 80.0;

    public static final double CRANE_INCHES_PER_COUNT   = (37.25/6817.0);//(38.0/7391.0);
    public static final double CRANE_CAL_POWER          = 1.0;
    public static final double CRANE_POWER              = 1.0;
    public static final double CRANE_SYNC_GAIN          = 0.01;
    public static final double CRANE_KP                 = 1.0;
    public static final double CRANE_KI                 = 0.0;
    public static final double CRANE_KD                 = 0.0;
    public static final double CRANE_KF                 = 0.0;
    public static final double CRANE_TOLERANCE          = 1.0;
    public static final double CRANE_SETTLING           = 0.2;
    public static final double CRANE_MIN_LENGTH         = 0.0;
    public static final double CRANE_MAX_LENGTH         = 90.0;

    public static final double TILTER_DEGREES_PER_COUNT = (16.0 / 6083.0);
    public static final double TILTER_CAL_POWER         = 0.25;
    public static final double TILTER_POWER             = 0.3;
    public static final double TILTER_DOWN_POWER_LIMIT  = -0.3;
    public static final double TILTER_UP_POWER_LIMIT    = 0.5;
    public static final double TILTER_KP                = 0.03;
    public static final double TILTER_KI                = 0.0;
    public static final double TILTER_KD                = 0.0;
    public static final double TILTER_KF                = 0.0;
    public static final double TILTER_TOLERANCE         = 1.0;
    public static final double TILTER_SETTLING          = 0.2;
    public static final double TILTER_MIN_ANGLE         = 0.0;
    public static final double TILTER_MAX_ANGLE         = 120.0;

    public static final double PICKUP_IN_POWER          = 0.5;
    public static final double PICKUP_OUT_POWER         = -1.0;

    //
    // Autonomous parameters.
    //
    public static final double AUTO_DISTANCE_TO_DEFENSE         = 50.0;
    public static final double AUTO_DISTANCE_CROSS_DEFENSE      = 130.0;

}   //class RobotInfo
