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
    public static final int CANID_LEFTFRONTMOTOR        = 3;    //Orange (40A)
    public static final int CANID_RIGHTFRONTMOTOR       = 4;    //Yellow (40A)
    public static final int CANID_LEFTREARMOTOR         = 5;    //Green (40A)
    public static final int CANID_RIGHTREARMOTOR        = 6;    //Blue (40A)
    public static final int CANID_LEFT_ELEVATOR         = 7;    //Purple (30A)
    public static final int CANID_RIGHT_ELEVATOR        = 8;    //Gray (30A)
    public static final int CANID_PICKUP                = 9;   //Orange (40A)
    public static final int CANID_WINCH                 = 10;    //White (40A)
    public static final int CANID_LEFT_ARM              = 11;   //Yellow (40A)
    public static final int CANID_RIGHT_ARM             = 12;   //Green (40A)
    public static final int CANID_FISHING_POLE          = 13;   //Blue (30A)
    public static final int CANID_POLE_TILTER           = 14;   //Purple (30A)
    
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
    public static final double DRIVEBASE_Y_SCALE        = 0.01621544056690844936881352331858;
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
    public static final double ELEVATOR_INCHES_PER_CLICK= (2.3562/1120);
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
