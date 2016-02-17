package frc492;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import frclib.FrcADXRS450Gyro;
import frclib.FrcCANTalon;
import frclib.FrcRobotBase;
import frclib.FrcRGBLight;
import hallib.HalDashboard;
import hallib.HalUtil;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcKalmanFilter;
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
    private static final String moduleName = "Robot";
    private TrcDbgTrace dbgTrace = null;

    private HalDashboard dashboard = HalDashboard.getInstance();
    private static final boolean visionTargetEnabled = false;
    private static final boolean debugDriveBase = false;
    private static final boolean debugPidDrive = false;
    private static final boolean debugPidElevator = true;
    private static final boolean debugPidSonar = false;
    private static boolean usbCameraEnabled = false;

//    public static boolean competitionRobot = true;

    //
    // Sensors
    //
    private BuiltInAccelerometer accelerometer;
    private TrcKalmanFilter accelXFilter;
    private TrcKalmanFilter accelYFilter;
    private TrcKalmanFilter accelZFilter;
//    private AnalogGyro gyro;
    private FrcADXRS450Gyro gyro;
    //
    // DriveBase subsystem.
    //
    public FrcCANTalon leftFrontMotor;
    public FrcCANTalon leftRearMotor;
    public FrcCANTalon rightFrontMotor;
    public FrcCANTalon rightRearMotor;
    public TrcDriveBase driveBase;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroTurnPidCtrl;
    public TrcPidDrive pidDrive;
    public TrcPidController sonarYPidCtrl;
    public TrcPidDrive sonarPidDrive;
    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public Elevator elevator;
    public Arm arm;
    public FrcCANTalon pickup;
    public FrcRGBLight rgbLight;

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
    public void initRobot()
    {
        final String funcName = "initRobot";

        //
        // Sensors.
        //
        accelerometer = new BuiltInAccelerometer();
        accelXFilter = new TrcKalmanFilter("accelX");
        accelYFilter = new TrcKalmanFilter("accelY");
        accelZFilter = new TrcKalmanFilter("accelZ");
        gyro = new FrcADXRS450Gyro();
//        gyro = new AnalogGyro(RobotInfo.AIN_GYRO);

        //
        // DriveBase subsystem.
        //
        leftFrontMotor = new FrcCANTalon(RobotInfo.CANID_LEFTFRONTMOTOR);
        leftRearMotor = new FrcCANTalon(RobotInfo.CANID_LEFTREARMOTOR);
        rightFrontMotor = new FrcCANTalon(RobotInfo.CANID_RIGHTFRONTMOTOR);
        rightRearMotor = new FrcCANTalon(RobotInfo.CANID_RIGHTREARMOTOR);

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
                "pidDrive", driveBase, null, encoderYPidCtrl, gyroTurnPidCtrl);

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
                null, sonarYPidCtrl, gyroTurnPidCtrl);

        //
        // Elevator subsystem.
        //
        elevator = new Elevator();

        //
        // Arm subsystem.
        //
        arm = new Arm();
        
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
        double xAccel = accelXFilter.filterData(accelerometer.getX());
        double yAccel = accelYFilter.filterData(accelerometer.getY());
        double zAccel = accelZFilter.filterData(accelerometer.getZ());
        HalDashboard.putNumber("Accel-X", xAccel);
        HalDashboard.putNumber("Accel-Y", yAccel);
        HalDashboard.putNumber("Accel-Z", zAccel);
        //
        // Elevator info.
        //
        HalDashboard.putNumber("Elevator Height", elevator.getHeight());
        HalDashboard.putBoolean("LeftLowerLimitSW", elevator.isLeftLowerLimitSwitchActive());
        HalDashboard.putBoolean("LeftUpperLimitSW", elevator.isLeftUpperLimitSwitchActive());
        HalDashboard.putBoolean("RightLowerLimitSW", elevator.isRightLowerLimitSwitchActive());
        HalDashboard.putBoolean("RightUpperLimitSW", elevator.isRightUpperLimitSwitchActive());
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
            encoderYPidCtrl.displayPidInfo(3);
            gyroTurnPidCtrl.displayPidInfo(5);
        }
        else if (debugPidElevator)
        {
            //
            // Elevator debug info.
            //
            dashboard.displayPrintf(
                    1, "ElevatorHeight=%5.1f",
                    elevator.getHeight());
            dashboard.displayPrintf(
                    2, "LeftLimit=%d/%d, RightLimit=%d/%d",
                    elevator.isLeftLowerLimitSwitchActive()? 1: 0,
                    elevator.isLeftUpperLimitSwitchActive()? 1: 0,
                    elevator.isRightLowerLimitSwitchActive()? 1: 0,
                    elevator.isRightUpperLimitSwitchActive()? 1: 0);
            elevator.displayDebugInfo(3);
        }
        else if (debugPidSonar)
        {
            sonarYPidCtrl.displayPidInfo(3);
            gyroTurnPidCtrl.displayPidInfo(5);
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

        if (pidCtrl == encoderYPidCtrl)
        {
            value = driveBase.getYPosition()*RobotInfo.DRIVEBASE_Y_SCALE;
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
