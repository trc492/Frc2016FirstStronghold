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
    // PWM channels.
    //

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTMOTOR        = 3;
    public static final int CANID_RIGHTFRONTMOTOR       = 4;
    public static final int CANID_LEFTREARMOTOR         = 5;
    public static final int CANID_RIGHTREARMOTOR        = 6;
    public static final int CANID_WINCH                 = 7;
    public static final int CANID_LEFT_ELEVATOR         = 8;
    public static final int CANID_RIGHT_ELEVATOR        = 9;
    public static final int CANID_LEFT_ARM              = 10;
    public static final int CANID_RIGHT_ARM             = 11;
    public static final int CANID_PICKUP                = 12;
    public static final int CANID_PDP                   = 16;
    public static final int CANID_PCM                   = 17;
    //
    // Analog channels.
    //
    public static final int AIN_GYRO                    = 0;
    public static final int AIN_ULTRASONIC              = 1;
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

    /**
     * Competition Robot.
     */
    //
    // DriveBase subsystem.
    //
    public static final boolean LEFTFRONTMOTOR_INVERTED = false;
    public static final boolean RIGHTFRONTMOTOR_INVERTED= true;
    public static final boolean LEFTREARMOTOR_INVERTED  = false;
    public static final boolean RIGHTREARMOTOR_INVERTED = true;
    public static final double XDRIVE_INCHES_PER_CLICK  = 0.01321074310684615838703943056885;
    public static final double YDRIVE_INCHES_PER_CLICK  = 0.01621544056690844936881352331858;
    public static final double X_RANGE_LIMIT            = 0.5;
    public static final double Y_RANGE_LIMIT            = 0.3;
    public static final double TURN_RANGE_LIMIT         = 0.35;
    public static final double SONAR_RANGE_LIMIT        = 0.2;
    public static final double X_KP                     = 0.045;
    public static final double X_KI                     = 0.0;
    public static final double X_KD                     = 0.0;
    public static final double X_KF                     = 0.0;
    public static final double X_TOLERANCE              = 1.0;
    public static final double X_SETTLING               = 0.2;
    public static final double Y_KP                     = 0.012;
    public static final double Y_KI                     = 0.0;
    public static final double Y_KD                     = 0.01;
    public static final double Y_KF                     = 0.0;
    public static final double Y_TOLERANCE              = 1.0;
    public static final double Y_SETTLING               = 0.2;
    public static final double TURN_KP                  = 0.016;
    public static final double TURN_KI                  = 0.0;
    public static final double TURN_KD                  = 0.0;
    public static final double TURN_KF                  = 0.0;
    public static final double TURN_TOLERANCE           = 1.0;
    public static final double TURN_SETTLING            = 0.2;
    public static final double SONAR_KP                 = 0.035;
    public static final double SONAR_KI                 = 0.0;
    public static final double SONAR_KD                 = 0.0;
    public static final double SONAR_KF                 = 0.0;
    public static final double SONAR_TOLERANCE          = 0.5;
    public static final double SONAR_SETTLING           = 0.2;
    public static final double DRIVE_SLOW_XSCALE        = 2.0;
    public static final double DRIVE_SLOW_YSCALE        = 3.0;
    public static final double DRIVE_SLOW_TURNSCALE     = 3.0;
    //
    // Elevator subsystem.
    //
    public static final double ELEVATOR_INCHES_PER_CLICK= 0.00386093837032573674406441520366;
    public static final double ELEVATOR_CAL_POWER       = -0.5;
    public static final byte ELEVATOR_SYNCGROUP         = 1;
    public static final double ELEVATOR_KP              = 0.3;
    public static final double ELEVATOR_KI              = 0.0;
    public static final double ELEVATOR_KD              = 0.0;
    public static final double ELEVATOR_KF              = 0.0;
    public static final double ELEVATOR_TOLERANCE       = 1.0;
    public static final double ELEVATOR_SETTLING        = 0.2;
    public static final double ELEVATOR_MIN_HEIGHT      = -1.0;
    public static final double ELEVATOR_MAX_HEIGHT      = 44.5;
    public static final double ELEVATOR_GROUND          = 0.0;
    public static final double ELEVATOR_1TOTE_HEIGHT    = 12.5;
    public static final double ELEVATOR_2TOTE_HEIGHT    = 25.0;
    public static final double ELEVATOR_LIFT_TOTE       = 6.0;
    public static final double ELEVATOR_STACK_TOTE      = 16.0;
    public static final double ELEVATOR_PICKUP_BIN      = 10.0;
    public static final double ELEVATOR_LIFT_BIN        = 15.0;
    public static final double ELEVATOR_STACK_BIN       = 25.0;
    public static final double ELEVATOR_HEIGHT_INC      = 1.0;

}   //class RobotInfo
