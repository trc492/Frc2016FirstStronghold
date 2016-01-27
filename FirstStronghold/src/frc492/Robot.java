package frc492;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import frclibj.TrcAnalogInput;
import frclibj.TrcDashboard;
import frclibj.TrcDbgTrace;
import frclibj.TrcDriveBase;
import frclibj.TrcKalmanFilter;
import frclibj.TrcMotorPosition;
import frclibj.TrcPidController;
import frclibj.TrcPidDrive;
import frclibj.TrcPneumatic;
import frclibj.TrcRGBLight;
import frclibj.TrcRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot
        extends TrcRobot
        implements TrcMotorPosition, TrcPidController.PidInput
{
    private static final String programName = "FirstStronghold";
    private static final String moduleName = "Robot";
    private static final boolean visionTargetEnabled = false;
    private static final boolean debugDriveBase = false;
    private static final boolean debugPidDrive = false;
    private static final boolean debugPidElevator = false;
    private static final boolean debugPidSonar = false;
    private static boolean usbCameraEnabled = false;
    private TrcDbgTrace dbgTrace = null;

//    public static boolean competitionRobot = true;

    //
    // Sensors
    //
    private BuiltInAccelerometer accelerometer;
    private TrcKalmanFilter accelXFilter;
    private TrcKalmanFilter accelYFilter;
    private TrcKalmanFilter accelZFilter;
    public double robotTilt;
    //
    // DriveBase subsystem.
    //
    public AnalogGyro gyro;
    public CANTalon leftFrontMotor;
    public CANTalon leftRearMotor;
    public CANTalon rightFrontMotor;
    public CANTalon rightRearMotor;
    public TrcDriveBase driveBase;
    public TrcPidController xPidCtrl;
    public TrcPidController yPidCtrl;
    public TrcPidController turnPidCtrl;
    public TrcPidDrive pidDrive;
    public TrcPidController sonarPidCtrl;
    public TrcPidDrive sonarPidDrive;
    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public Elevator elevator;
    public TrcPneumatic lowerGrabber;
    public TrcPneumatic upperGrabber;
    public TrcPneumatic pusher;
    public TrcRGBLight rgbLight;

    //
    // Vision target subsystem.
    //
    public VisionTarget visionTarget = null;
    //
    // Camera subsystem.
    //
    private CameraServer cameraServer = null;
    private int usbCamSession = -1;
    private Image usbCamImage = null;
    private double nextCaptureTime = Timer.getFPGATimestamp();
    private static final String targetLeftKey = "Target Left";
    private static final String targetRightKey = "Target Right";
    private static final String targetTopKey = "Target Top";
    private static final String targetBottomKey = "TargetBottom";
    private static int targetLeft = 170;
    private static int targetRight = 520;
    private static int targetTop = 130;
    private static int targetBottom = 440;
    private static Rect targetRect =
            new Rect(targetTop,
                     targetLeft,
                     targetBottom - targetTop,
                     targetRight - targetLeft);
    //
    // Robot Modes.
    //
    private RobotMode teleOpMode;
    private RobotMode autoMode;
    private RobotMode testMode;
    //
    // Ultrasonic subsystem (has a dependency on teleOpMode).
    //
    public TrcAnalogInput ultrasonic;
    private TrcKalmanFilter sonarFilter;
    public double sonarDistance;
    private static final String dispenserDistanceKey = "Dispenser Distance";
    public double dispenserDistance = 26.0;

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
        dbgTrace = new TrcDbgTrace(
                moduleName,
                false,
                TrcDbgTrace.TraceLevel.API,
                TrcDbgTrace.MsgLevel.INFO);
    }   //Robot

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        final String funcName = "robotInit";
        //
        // Sensors.
        //
        accelerometer = new BuiltInAccelerometer();
        accelXFilter = new TrcKalmanFilter();
        accelYFilter = new TrcKalmanFilter();
        accelZFilter = new TrcKalmanFilter();
        //
        // DriveBase subsystem.
        //
        gyro = new AnalogGyro(RobotInfo.AIN_GYRO);
        leftFrontMotor = new CANTalon(RobotInfo.CANID_LEFTFRONTMOTOR);
        leftRearMotor = new CANTalon(RobotInfo.CANID_LEFTREARMOTOR);
        rightFrontMotor = new CANTalon(RobotInfo.CANID_RIGHTFRONTMOTOR);
        rightRearMotor = new CANTalon(RobotInfo.CANID_RIGHTREARMOTOR);
        //
        // Initialize each drive motor controller.
        //
        leftFrontMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        leftRearMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightFrontMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightRearMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        leftFrontMotor.reverseSensor(RobotInfo.LEFTFRONTMOTOR_INVERTED);
        leftRearMotor.reverseSensor(RobotInfo.LEFTREARMOTOR_INVERTED);
        rightFrontMotor.reverseSensor(RobotInfo.RIGHTFRONTMOTOR_INVERTED);
        rightRearMotor.reverseSensor(RobotInfo.RIGHTREARMOTOR_INVERTED);
        //
        // Reset encoders.
        //
        leftFrontMotor.setPosition(0.0);
        leftRearMotor.setPosition(0.0);
        rightFrontMotor.setPosition(0.0);
        rightRearMotor.setPosition(0.0);
        //
        // Initialize DriveBase subsystem.
        //
        driveBase = new TrcDriveBase(
                leftFrontMotor,
                leftRearMotor,
                rightFrontMotor,
                rightRearMotor,
                this,
                gyro);
        driveBase.setInvertedMotor(
                TrcDriveBase.MotorType.kFrontLeft,
                RobotInfo.LEFTFRONTMOTOR_INVERTED);
        driveBase.setInvertedMotor(
                TrcDriveBase.MotorType.kRearLeft,
                RobotInfo.LEFTREARMOTOR_INVERTED);
        driveBase.setInvertedMotor(
                TrcDriveBase.MotorType.kFrontRight,
                RobotInfo.RIGHTFRONTMOTOR_INVERTED);
        driveBase.setInvertedMotor(
                TrcDriveBase.MotorType.kRearRight,
                RobotInfo.RIGHTREARMOTOR_INVERTED);
        //
        // Create PID controllers for DriveBase PID drive.
        //
        xPidCtrl = new TrcPidController(
                "xPidCtrl",
                RobotInfo.X_KP,
                RobotInfo.X_KI,
                RobotInfo.X_KD,
                RobotInfo.X_KF,
                RobotInfo.X_TOLERANCE,
                RobotInfo.X_SETTLING,
                this,
                0);
        yPidCtrl = new TrcPidController(
                "yPidCtrl",
                RobotInfo.Y_KP,
                RobotInfo.Y_KI,
                RobotInfo.Y_KD,
                RobotInfo.Y_KF,
                RobotInfo.Y_TOLERANCE,
                RobotInfo.Y_SETTLING,
                this,
                0);
        turnPidCtrl = new TrcPidController(
                "turnPidCtrl",
                RobotInfo.TURN_KP,
                RobotInfo.TURN_KI,
                RobotInfo.TURN_KD,
                RobotInfo.TURN_KF,
                RobotInfo.TURN_TOLERANCE,
                RobotInfo.TURN_SETTLING,
                this,
                0);
        pidDrive = new TrcPidDrive(
                "pidDrive",
                driveBase,
                xPidCtrl,
                yPidCtrl,
                turnPidCtrl);

        sonarPidCtrl = new TrcPidController(
                "sonarPidCtrl",
                RobotInfo.SONAR_KP,
                RobotInfo.SONAR_KI,
                RobotInfo.SONAR_KD,
                RobotInfo.SONAR_KF,
                RobotInfo.SONAR_TOLERANCE,
                RobotInfo.SONAR_SETTLING,
                this,
                (TrcPidController.PIDCTRLO_ABS_SETPT |
                 TrcPidController.PIDCTRLO_INVERTED));
        sonarPidDrive =
                new TrcPidDrive(
                        "sonarPidDrive",
                        driveBase,
                        xPidCtrl,
                        sonarPidCtrl,
                        turnPidCtrl);

        //
        // Elevator subsystem.
        //
        elevator = new Elevator();
        //
        // RGB LED light
        //
        if (RobotInfo.ENABLE_LEDS)
        {
            rgbLight = new TrcRGBLight(
                    "rgbLight",
                    RobotInfo.CANID_PCM,
                    RobotInfo.SOL_LED_RED,
                    RobotInfo.SOL_LED_GREEN,
                    RobotInfo.SOL_LED_BLUE);
        }
        else
        {
            rgbLight = null;
        }
        //
        // Vision subsystem.
        //
        if (visionTargetEnabled)
        {
            visionTarget = new VisionTarget();
        }
        else
        {
            visionTarget = null;
        }
        //
        // Camera subsystem for streaming.
        //
        if (usbCameraEnabled)
        {
            try
            {
                usbCamSession = NIVision.IMAQdxOpenCamera(
                        RobotInfo.USB_CAM_NAME,
                        IMAQdxCameraControlMode.CameraControlModeController);
                NIVision.IMAQdxConfigureGrab(usbCamSession);
                NIVision.IMAQdxStartAcquisition(usbCamSession);

                usbCamImage = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
                cameraServer = CameraServer.getInstance();
                cameraServer.setQuality(30);
            }
            catch (Exception e)
            {
                usbCameraEnabled = false;
                dbgTrace.traceWarn(
                        funcName, "Failed to open USB camera, disable it!");
            }
        }

        //
        // Robot Modes.
        //
        teleOpMode = new TeleOp(this);
        autoMode = new Autonomous(this);
        testMode = new Test();
        setupRobotModes(teleOpMode, autoMode, testMode, null);

        ultrasonic =
                new TrcAnalogInput(
                        "frontSonar",
                        RobotInfo.AIN_ULTRASONIC,
                        RobotInfo.ULTRASONIC_INCHESPERVOLT,
                        dispenserDistance - 1.0,
                        dispenserDistance + 1.0,
                        0,
                        (TrcAnalogInput.AnalogEventHandler)teleOpMode);
        sonarFilter = new TrcKalmanFilter();

        dispenserDistance =
                TrcDashboard.getNumber(dispenserDistanceKey, dispenserDistance);
    }   //robotInit

    public void updateDashboard()
    {
        //
        // Sensor info.
        //
        sonarDistance = sonarFilter.filter(ultrasonic.getData());
        TrcDashboard.putNumber("Sonar Distance", sonarDistance);
        double xAccel = accelXFilter.filter(accelerometer.getX());
        double yAccel = accelYFilter.filter(accelerometer.getY());
        double zAccel = accelZFilter.filter(accelerometer.getZ());
        TrcDashboard.putNumber("Accel-X", xAccel);
        TrcDashboard.putNumber("Accel-Y", yAccel);
        TrcDashboard.putNumber("Accel-Z", zAccel);
        robotTilt = zAccel;
        if (yAccel < 0.0)
        {
            robotTilt = -robotTilt;
        }
        robotTilt = 180.0*Math.asin(robotTilt)/Math.PI;
        TrcDashboard.putNumber("Tilt", robotTilt);
        //
        // Elevator info.
        //
        TrcDashboard.putNumber(
                "Elevator Height", elevator.getHeight());
        TrcDashboard.putBoolean(
                "LowerLimitSW", !elevator.isLowerLimitSwitchActive());
        TrcDashboard.putBoolean(
                "UpperLimitSW", !elevator.isUpperLimitSwitchActive());
        //
        // USB camera streaming.
        //
        if (usbCameraEnabled && Timer.getFPGATimestamp() >= nextCaptureTime)
        {
            //
            // Capture at 10fps.
            //
            nextCaptureTime = Timer.getFPGATimestamp() + 0.1;
            NIVision.IMAQdxGrab(usbCamSession, usbCamImage, 1);
            targetLeft = (int)
                    TrcDashboard.getNumber(targetLeftKey, targetLeft);
            targetRight = (int)
                    TrcDashboard.getNumber(targetRightKey, targetRight);
            targetTop = (int)
                    TrcDashboard.getNumber(targetTopKey, targetTop);
            targetBottom = (int)
                    TrcDashboard.getNumber(targetBottomKey, targetBottom);
            targetRect.left = targetLeft;
            targetRect.top = targetTop;
            targetRect.width = targetRight - targetLeft;
            targetRect.height = targetBottom - targetTop;
            NIVision.imaqDrawShapeOnImage(
                    usbCamImage,
                    usbCamImage,
                    targetRect,
                    DrawMode.DRAW_VALUE,
                    ShapeMode.SHAPE_RECT,
                    (float)0x0);
            cameraServer.setImage(usbCamImage);
        }

        if (debugDriveBase)
        {
            //
            // DriveBase debug info.
            //
            TrcDashboard.textPrintf(
                    1, "LFEnc=%8.0f, RFEnc=%8.0f",
                    leftFrontMotor.getPosition(),
                    rightFrontMotor.getPosition());
            TrcDashboard.textPrintf(
                    2, "LREnc=%8.0f, RREnc=%8.0f",
                    leftRearMotor.getPosition(),
                    rightRearMotor.getPosition());
            TrcDashboard.textPrintf(
                    3, "XPos=%6.1f, YPos=%6.1f, Heading=%6.1f",
                    driveBase.getXPosition()*RobotInfo.XDRIVE_INCHES_PER_CLICK,
                    driveBase.getYPosition()*RobotInfo.YDRIVE_INCHES_PER_CLICK,
                    driveBase.getHeading());
            TrcDashboard.textPrintf(
                    4, "Ultrasonic=%5.1f", sonarDistance);
        }
        else if (debugPidDrive)
        {
            xPidCtrl.displayPidInfo(1);
            yPidCtrl.displayPidInfo(3);
            turnPidCtrl.displayPidInfo(5);
        }
        else if (debugPidElevator)
        {
            //
            // Elevator debug info.
            //
            TrcDashboard.textPrintf(
                    1, "ElevatorHeight=%5.1f",
                    elevator.getHeight());
            TrcDashboard.textPrintf(
                    2, "LowerLimit=%s, UpperLimit=%s",
                    Boolean.toString(elevator.isLowerLimitSwitchActive()),
                    Boolean.toString(elevator.isUpperLimitSwitchActive()));
            elevator.displayDebugInfo(3);
        }
        else if (debugPidSonar)
        {
            xPidCtrl.displayPidInfo(1);
            sonarPidCtrl.displayPidInfo(3);
            turnPidCtrl.displayPidInfo(5);
        }
    }   //updateDashboard

    //
    // Implements TrcDriveBase.MotorPosition.
    //
    public double getMotorPosition(SpeedController speedController)
    {
        return ((CANTalon)speedController).getPosition();
    }   //getMotorPosition

    public double getMotorSpeed(SpeedController speedController)
    {
        return ((CANTalon)speedController).getSpeed()*10.0;
    }   //getMotorSpeed

    public void resetMotorPosition(SpeedController speedController)
    {
        ((CANTalon)speedController).setPosition(0.0);
    }   //resetMotorPosition

    public void reversePositionSensor(
            SpeedController speedController,
            boolean flip)
    {
        ((CANTalon)speedController).reverseSensor(flip);
    }   //reversePositionSensor

    public boolean isForwardLimitSwitchActive(SpeedController speedController)
    {
        //
        // There is no limit switches on the DriveBase.
        //
        return false;
    }   //isForwardLimitSwitchActive

    public boolean isReverseLimitSwitchActive(SpeedController speedController)
    {
        //
        // There is no limit switches on the DriveBase.
        //
        return false;
    }   //isReverseLimitSwitchActive

    //
    // Implements TrcPidController.PidInput.
    //
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == xPidCtrl)
        {
            value = driveBase.getXPosition()*RobotInfo.XDRIVE_INCHES_PER_CLICK;
        }
        else if (pidCtrl == yPidCtrl)
        {
            value = driveBase.getYPosition()*RobotInfo.YDRIVE_INCHES_PER_CLICK;
        }
        else if (pidCtrl == turnPidCtrl)
        {
            value = gyro.getAngle();
        }
        else if (pidCtrl == sonarPidCtrl)
        {
            value = sonarDistance;
        }

        return value;
    }   //getInput

}   //class Robot
