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
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTMOTOR        = 3;    //40A: Orange
    public static final int CANID_RIGHTFRONTMOTOR       = 4;    //40A: Yellow
    public static final int CANID_LEFTREARMOTOR         = 5;    //40A: Green
    public static final int CANID_RIGHTREARMOTOR        = 6;    //40A: Blue
    public static final int CANID_CRANE                 = 7;    //40A: Purple
    public static final int CANID_TILTER                = 8;    //40A: Gray
    public static final int CANID_RESERVED1             = 9;    //40A: White
    public static final int CANID_RESERVED2             = 10;   //30A: Orange
    public static final int CANID_LEFT_ARM              = 11;   //30A: Yellow
    public static final int CANID_RIGHT_ARM             = 12;   //30A: Green
    public static final int CANID_PICKUP                = 13;   //40A: Blue
    public static final int CANID_WINCH                 = 14;   //40A: Purple
    
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
    public static final String USB_CAM_NAME             = "cam1";

    //
    // DriveBase subsystem.
    //
    public static final double DRIVEBASE_X_SCALE        = 75.695968948315512041060994714416;
    public static final double ENCODER_X_KP             = 0.045;
    public static final double ENCODER_X_KI             = 0.0;
    public static final double ENCODER_X_KD             = 0.0;
    public static final double ENCODER_X_KF             = 0.0;
    public static final double ENCODER_X_TOLERANCE      = 1.0;
    public static final double ENCODER_X_SETTLING       = 0.2;
    public static final double DRIVEBASE_Y_SCALE        = 61.669616429709793669719405297833;
    public static final double ENCODER_Y_KP             = 0.018;
    public static final double ENCODER_Y_KI             = 0.0;
    public static final double ENCODER_Y_KD             = 0.01;
    public static final double ENCODER_Y_KF             = 0.0;
    public static final double ENCODER_Y_TOLERANCE      = 1.0;
    public static final double ENCODER_Y_SETTLING       = 0.2;
    public static final double GYRO_TURN_KP             = 0.010;
    public static final double GYRO_TURN_KI             = 0.0;
    public static final double GYRO_TURN_KD             = 0.0;
    public static final double GYRO_TURN_KF             = 0.0;
    public static final double GYRO_TURN_TOLERANCE      = 1.0;
    public static final double GYRO_TURN_SETTLING       = 0.2;
    public static final double SONAR_Y_KP               = 0.035;
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
    public static final double ARM_COUNTS_PER_DEGREE    = 0.55; //(100/180)
    public static final double ARM_CAL_POWER            = 0.3;
    public static final double ARM_SYNC_GAIN            = 0.05;
    public static final double ARM_KP                   = 0.01;
    public static final double ARM_KI                   = 0.0;
    public static final double ARM_KD                   = 0.0;
    public static final double ARM_KF                   = 0.0;
    public static final double ARM_TOLERANCE            = 1.0;
    public static final double ARM_SETTLING             = 0.2;
    public static final double ARM_UP_POSITION          = 0.0;
    public static final double ARM_DOWN_POSITION        = 44.5;
 
    //
    // Crane subsystem.
    //
    public static final double WINCH_COUNTS_PER_INCH    = (2.3562/1120);
    public static final double WINCH_MAX_LENGTH         = 80.0;

    public static final double CRANE_COUNTS_PER_INCH    = (2.3562/1120);
    public static final double CRANE_CAL_POWER          = 0.3;
    public static final double CRANE_SYNC_GAIN          = 0.01;
    public static final double CRANE_KP                 = 1.0;
    public static final double CRANE_KI                 = 0.0;
    public static final double CRANE_KD                 = 0.0;
    public static final double CRANE_KF                 = 0.0;
    public static final double CRANE_TOLERANCE          = 1.0;
    public static final double CRANE_SETTLING           = 0.2;
    public static final double CRANE_MIN_LENGTH         = 0.0;
    public static final double CRANE_MAX_LENGTH         = 90.0;

    public static final double TILTER_COUNTS_PER_DEGREE = (2.3562/1120);
    public static final double TILTER_CAL_POWER         = 0.3;
    public static final double TILTER_KP                = 1.0;
    public static final double TILTER_KI                = 0.0;
    public static final double TILTER_KD                = 0.0;
    public static final double TILTER_KF                = 0.0;
    public static final double TILTER_TOLERANCE         = 1.0;
    public static final double TILTER_SETTLING          = 0.2;
    public static final double TILTER_MIN_ANGLE         = 0.0;
    public static final double TILTER_MAX_ANGLE         = 90.0;

}   //class RobotInfo
