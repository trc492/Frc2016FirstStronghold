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
    // Analog channels.
    //
    public static final int AIN_ULTRASONIC              = 0;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTMOTOR        = 3;    //40A: Orange
    public static final int CANID_RIGHTFRONTMOTOR       = 4;    //40A: Yellow
    public static final int CANID_LEFTREARMOTOR         = 5;    //40A: Green
    public static final int CANID_RIGHTREARMOTOR        = 6;    //40A: Blue
    public static final int CANID_CRANE_TILTER          = 7;    //40A: Purple
    public static final int CANID_CRANE                 = 8;    //40A: Gray
    public static final int CANID_LEFT_ELEVATOR         = 9;    //40A: White
    public static final int CANID_RIGHT_ELEVATOR        = 10;   //30A: Orange
    public static final int CANID_LEFT_ARM              = 11;   //30A: Yellow
    public static final int CANID_RIGHT_ARM             = 12;   //30A: Green
    public static final int CANID_PICKUP                = 13;   //40A: Blue
    public static final int CANID_SPARE                 = 14;   //40A: Purple
    
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
    public static final double DRIVEBASE_Y_SCALE        = 61.669616429709793669719405297833;
    public static final double ENCODER_Y_KP             = 0.012;
    public static final double ENCODER_Y_KI             = 0.0;
    public static final double ENCODER_Y_KD             = 0.01;
    public static final double ENCODER_Y_KF             = 0.0;
    public static final double ENCODER_Y_TOLERANCE      = 1.0;
    public static final double ENCODER_Y_SETTLING       = 0.2;
    public static final double GYRO_TURN_KP             = 0.016;
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
    public static final double DRIVE_SLOW_YSCALE        = 3.0;
    public static final double DRIVE_SLOW_TURNSCALE     = 3.0;

    //
    // Elevator subsystem.
    //
    public static final double ELEVATOR_COUNTS_PER_INCH = (2.3562/1120);
    public static final double ELEVATOR_CAL_POWER       = -0.5;
    public static final byte   ELEVATOR_SYNCGROUP       = 1;
    public static final double ELEVATOR_SYNC_GAIN       = 0.003;
    public static final double ELEVATOR_KP              = 0.3;
    public static final double ELEVATOR_KI              = 0.0;
    public static final double ELEVATOR_KD              = 0.0;
    public static final double ELEVATOR_KF              = 0.0;
    public static final double ELEVATOR_TOLERANCE       = 1.0;
    public static final double ELEVATOR_SETTLING        = 0.2;
    public static final double ELEVATOR_MIN_HEIGHT      = -1.0;
    public static final double ELEVATOR_MAX_HEIGHT      = 44.5;
    public static final double ELEVATOR_GROUND          = 0.0;

    //
    // Arm subsystem.
    //
    public static final double ARM_SCALE           = (2.3562/1120);
    public static final byte   ARM_SYNCGROUP       = 2;
    public static final double ARM_SYNC_GAIN       = 0.003;
    public static final double ARM_KP              = 0.3;
    public static final double ARM_KI              = 0.0;
    public static final double ARM_KD              = 0.0;
    public static final double ARM_KF              = 0.0;
    public static final double ARM_TOLERANCE       = 1.0;
    public static final double ARM_SETTLING        = 0.2;
    public static final double ARM_MIN_POSITION    = -1.0;
    public static final double ARM_MAX_POSITION    = 44.5;
    public static final double ARM_UP_DELTA        = 5.0;
    public static final double ARM_DOWN_DELTA      = -5.0;
 
}   //class RobotInfo
