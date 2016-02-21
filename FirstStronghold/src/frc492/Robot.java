package frc492;

import edu.wpi.first.wpilibj.CANTalon;
import frclib.FrcADXRS450Gyro;
import frclib.FrcCANTalon;
import frclib.FrcRobotBase;
import frclib.FrcRGBLight;
import hallib.HalDashboard;
import trclib.TrcDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot.RobotMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends FrcRobotBase implements TrcPidController.PidInput
{
    private static final String programName = "FirstStronghold";
//    private static final String moduleName = "Robot";
//    private TrcDbgTrace dbgTrace = FrcRobotBase.getRobotTracer();

    private HalDashboard dashboard = HalDashboard.getInstance();
//    private static final boolean visionTargetEnabled = false;
//    private static boolean usbCameraEnabled = false;
    private static final boolean debugDriveBase = false;
    private static final boolean debugPidDrive = false;
    private static final boolean debugPidSonar = false;

//    public static boolean competitionRobot = true;

    //
    // Sensors
    //
    private FrcADXRS450Gyro gyro;
    //
    // DriveBase subsystem.
    //
    public FrcCANTalon leftFrontMotor;
    public FrcCANTalon leftRearMotor;
    public FrcCANTalon rightFrontMotor;
    public FrcCANTalon rightRearMotor;
    public TrcDriveBase driveBase;
    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroTurnPidCtrl;
    public TrcPidDrive pidDrive;
    public TrcPidController sonarYPidCtrl;
    public TrcPidDrive sonarPidDrive;
    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public Arm arm;
    public Crane crane;
    public FrcCANTalon pickup;
    public FrcRGBLight rgbLight;

    /*
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
    private double nextCaptureTime = HalUtil.getCurrentTime();
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
                     */
    //
    // Robot Modes.
    //
    private RobotMode teleOpMode;
    private RobotMode autoMode;
    private RobotMode testMode;
    //
    // Ultrasonic subsystem (has a dependency on teleOpMode).
    //
    /*
    public TrcAnalogInput ultrasonic;
    private TrcKalmanFilter sonarFilter;
    public double sonarDistance;
    private static final String wallDistanceKey = "Wall Distance";
    public double wallDistance = 26.0;
    */

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
    }   //Robot

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void initRobot()
    {
        //
        // Sensors.
        //
        gyro = new FrcADXRS450Gyro();

        //
        // DriveBase subsystem.
        //
        leftFrontMotor = new FrcCANTalon(RobotInfo.CANID_LEFTFRONTMOTOR);
        leftRearMotor = new FrcCANTalon(RobotInfo.CANID_LEFTREARMOTOR);
        rightFrontMotor = new FrcCANTalon(RobotInfo.CANID_RIGHTFRONTMOTOR);
        rightRearMotor = new FrcCANTalon(RobotInfo.CANID_RIGHTREARMOTOR);
        leftFrontMotor.setInverted(false);
        leftRearMotor.setInverted(false);
        rightFrontMotor.setInverted(true);
        rightRearMotor.setInverted(true);
        leftFrontMotor.setPositionSensorInverted(true);
        leftRearMotor.setPositionSensorInverted(true);
        rightFrontMotor.setPositionSensorInverted(false);
        rightRearMotor.setPositionSensorInverted(false);

        //
        // Initialize each drive motor controller.
        //
        leftFrontMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        leftRearMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightFrontMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightRearMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

        //
        // Initialize DriveBase subsystem.
        //
        driveBase = new TrcDriveBase(
                leftFrontMotor,
                leftRearMotor,
                rightFrontMotor,
                rightRearMotor,
                gyro);

        //
        // Create PID controllers for DriveBase PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                RobotInfo.ENCODER_X_KP,
                RobotInfo.ENCODER_X_KI,
                RobotInfo.ENCODER_X_KD,
                RobotInfo.ENCODER_X_KF,
                RobotInfo.ENCODER_X_TOLERANCE,
                RobotInfo.ENCODER_X_SETTLING,
                this);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                RobotInfo.ENCODER_Y_KP,
                RobotInfo.ENCODER_Y_KI,
                RobotInfo.ENCODER_Y_KD,
                RobotInfo.ENCODER_Y_KF,
                RobotInfo.ENCODER_Y_TOLERANCE,
                RobotInfo.ENCODER_Y_SETTLING,
                this);
        gyroTurnPidCtrl = new TrcPidController(
                "gyroTurnPidCtrl",
                RobotInfo.GYRO_TURN_KP,
                RobotInfo.GYRO_TURN_KI,
                RobotInfo.GYRO_TURN_KD,
                RobotInfo.GYRO_TURN_KF,
                RobotInfo.GYRO_TURN_TOLERANCE,
                RobotInfo.GYRO_TURN_SETTLING,
                this);
        pidDrive = new TrcPidDrive(
                "pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);

        sonarYPidCtrl = new TrcPidController(
                "sonarYPidCtrl",
                RobotInfo.SONAR_Y_KP,
                RobotInfo.SONAR_Y_KI,
                RobotInfo.SONAR_Y_KD,
                RobotInfo.SONAR_Y_KF,
                RobotInfo.SONAR_Y_TOLERANCE,
                RobotInfo.SONAR_Y_SETTLING,
                this);
        sonarYPidCtrl.setAbsoluteSetPoint(true);
        sonarYPidCtrl.setInverted(true);;
        sonarPidDrive = new TrcPidDrive(
                "sonarPidDrive", driveBase,
                encoderXPidCtrl, sonarYPidCtrl, gyroTurnPidCtrl);

        //
        // Arm subsystem.
        //
        arm = new Arm();
        
        //
        // Crane subsystem.
        //
        crane = new Crane();
        
        //
        // Pickup subsystem
        //
        pickup = new FrcCANTalon(RobotInfo.CANID_PICKUP);
        
        //
        // RGB LED light
        //
        if (RobotInfo.ENABLE_LEDS)
        {
            rgbLight = new FrcRGBLight(
                    "rgbLight",
                    RobotInfo.CANID_PCM1,
                    RobotInfo.SOL_LED_RED,
                    RobotInfo.SOL_LED_GREEN,
                    RobotInfo.SOL_LED_BLUE);
        }
        else
        {
            rgbLight = null;
        }

        /*
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
        */

        //
        // Robot Modes.
        //
        teleOpMode = new TeleOp(this);
        autoMode = new Autonomous(this);
        testMode = new Test(this);
        setupRobotModes(teleOpMode, autoMode, testMode, null);

        /*
        ultrasonic =
                new TrcAnalogInput(
                        "frontSonar",
                        RobotInfo.AIN_ULTRASONIC,
                        RobotInfo.ULTRASONIC_INCHESPERVOLT,
                        wallDistance - 1.0,
                        wallDistance + 1.0,
                        0,
                        (TrcAnalogInput.AnalogEventHandler)teleOpMode);
        sonarFilter = new TrcKalmanFilter();

        dispenserDistance =
                TrcDashboard.getNumber(dispenserDistanceKey, dispenserDistance);
                */
    }   //initRobot

    public void updateDashboard()
    {
        //
        // Sensor info.
        //
        /*
        sonarDistance = sonarFilter.filter(ultrasonic.getData());
        TrcDashboard.putNumber("Sonar Distance", sonarDistance);
        */
        /*
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
                    HalDashboard.getNumber(targetLeftKey, targetLeft);
            targetRight = (int)
                    HalDashboard.getNumber(targetRightKey, targetRight);
            targetTop = (int)
                    HalDashboard.getNumber(targetTopKey, targetTop);
            targetBottom = (int)
                    HalDashboard.getNumber(targetBottomKey, targetBottom);
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
        */

        if (debugDriveBase)
        {
            //
            // DriveBase debug info.
            //
            dashboard.displayPrintf(
                    1, "LFEnc=%8.0f, RFEnc=%8.0f",
                    leftFrontMotor.getPosition(),
                    rightFrontMotor.getPosition());
            dashboard.displayPrintf(
                    2, "LREnc=%8.0f, RREnc=%8.0f",
                    leftRearMotor.getPosition(),
                    rightRearMotor.getPosition());
            dashboard.displayPrintf(
                    3, "YPos=%6.1f, Heading=%6.1f",
                    driveBase.getYPosition()*RobotInfo.DRIVEBASE_Y_SCALE,
                    driveBase.getHeading());
        }
        else if (debugPidDrive)
        {
            encoderXPidCtrl.displayPidInfo(3);
            encoderYPidCtrl.displayPidInfo(5);
            gyroTurnPidCtrl.displayPidInfo(7);
        }
        else if (debugPidSonar)
        {
            encoderXPidCtrl.displayPidInfo(3);
            sonarYPidCtrl.displayPidInfo(5);
            gyroTurnPidCtrl.displayPidInfo(7);
        }
    }   //updateDashboard

    //
    // Implements TrcPidController.PidInput.
    //
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == encoderXPidCtrl)
        {
            value = driveBase.getXPosition()/RobotInfo.DRIVEBASE_X_SCALE;
        }
        else if (pidCtrl == encoderYPidCtrl)
        {
            value = driveBase.getYPosition()/RobotInfo.DRIVEBASE_Y_SCALE;
        }
        else if (pidCtrl == gyroTurnPidCtrl)
        {
            value = gyro.getAngle();
        }
        /*
        else if (pidCtrl == sonarYPidCtrl)
        {
            value = sonarDistance;
        }
        */

        return value;
    }   //getInput

}   //class Robot
