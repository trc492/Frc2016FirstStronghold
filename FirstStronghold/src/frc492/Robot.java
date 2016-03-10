package frc492;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.IMAQdxCameraControlMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import frclib.FrcADXRS450Gyro;
import frclib.FrcCANTalon;
import frclib.FrcDigitalRGB;
import frclib.FrcRobotBase;
import frclib.FrcVision;
import hallib.HalDashboard;
import hallib.HalUtil;
import trclib.TrcDbgTrace;
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
public class Robot extends FrcRobotBase implements TrcPidController.PidInput,
                                                   FrcVision.ImageProvider
{
    private static final String programName = "FirstStronghold";
    private TrcDbgTrace dbgTrace = FrcRobotBase.getRobotTracer();

    private static final boolean usbCameraEnabled = true;
    private static final boolean visionTargetEnabled = false;
    private static final boolean debugDriveBase = true;
    private static final boolean debugArm = true;
    private static final boolean debugCrane = true;
    private static final boolean debugPidDrive = false;
    private static final boolean debugPidSonar = false;
    private static final double DASHBOARD_UPDATE_INTERVAL = 0.1;

    private HalDashboard dashboard = HalDashboard.getInstance();
    private double nextUpdateTime = HalUtil.getCurrentTime();

    //
    // Sensors.
    //
    private FrcADXRS450Gyro gyro;
//    private TrcAnalogInput sonar;
//    private TrcKalmanFilter sonarFilter;

    //
    // Camera.
    //
    private CameraServer cameraServer = null;
    private int usbCamSession = -1;
    private Image usbCamImage = null;
    private Image overlayImage = null;
    private boolean freshImage = false;
    private boolean videoEnabled = false;

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
    public FrcDigitalRGB leftLight;
    public FrcDigitalRGB rightLight;

    //
    // Vision target subsystem.
    //
    public VisionTarget visionTarget = null;
    /*
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
        final String funcName = "initRobot";

        //
        // Sensors.
        //
        gyro = new FrcADXRS450Gyro();
        /*
        sonar = new TrcAnalogInput(
                        "frontSonar",
                        RobotInfo.AIN_ULTRASONIC,
                        RobotInfo.ULTRASONIC_INCHESPERVOLT,
                        wallDistance - 1.0,
                        wallDistance + 1.0,
                        0,
                        (TrcAnalogInput.AnalogEventHandler)teleOpMode);
        sonarFilter = new TrcKalmanFilter();
        */

        //
        // Camera for streaming.
        //
        if (usbCameraEnabled)
        {
            try
            {
                usbCamSession = NIVision.IMAQdxOpenCamera(
                        RobotInfo.USB_CAM_NAME,
                        IMAQdxCameraControlMode.CameraControlModeController);
                NIVision.IMAQdxConfigureGrab(usbCamSession);

                usbCamImage = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
                overlayImage = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
                cameraServer = CameraServer.getInstance();
                cameraServer.setQuality(30);
            }
            catch (Exception e)
            {
                cameraServer = null;
                dbgTrace.traceWarn(
                        funcName, "Failed to open USB camera, disable it!");
            }
        }

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
        leftFrontMotor.setInverted(false);
        leftRearMotor.setInverted(false);
        rightFrontMotor.setInverted(true);
        rightRearMotor.setInverted(true);

        leftFrontMotor.setVoltageRampRate(10.0);
        leftRearMotor.setVoltageRampRate(10.0);
        rightFrontMotor.setVoltageRampRate(10.0);
        rightRearMotor.setVoltageRampRate(10.0);

        leftFrontMotor.setPositionSensorInverted(true);
        leftRearMotor.setPositionSensorInverted(true);
        rightFrontMotor.setPositionSensorInverted(false);
        rightRearMotor.setPositionSensorInverted(false);

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
        encoderXPidCtrl.setAbsoluteSetPoint(true);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                RobotInfo.ENCODER_Y_KP,
                RobotInfo.ENCODER_Y_KI,
                RobotInfo.ENCODER_Y_KD,
                RobotInfo.ENCODER_Y_KF,
                RobotInfo.ENCODER_Y_TOLERANCE,
                RobotInfo.ENCODER_Y_SETTLING,
                this);
        encoderYPidCtrl.setAbsoluteSetPoint(true);
        gyroTurnPidCtrl = new TrcPidController(
                "gyroTurnPidCtrl",
                RobotInfo.GYRO_TURN_KP,
                RobotInfo.GYRO_TURN_KI,
                RobotInfo.GYRO_TURN_KD,
                RobotInfo.GYRO_TURN_KF,
                RobotInfo.GYRO_TURN_TOLERANCE,
                RobotInfo.GYRO_TURN_SETTLING,
                this);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
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
        sonarYPidCtrl.setInverted(true);
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
            leftLight = new FrcDigitalRGB(
                    "leftLight",
                    RobotInfo.DOUT_LEFTLIGHT_RED,
                    RobotInfo.DOUT_LEFTLIGHT_GREEN,
                    RobotInfo.DOUT_LEFTLIGHT_BLUE);
            rightLight = new FrcDigitalRGB(
                    "rightLight",
                    RobotInfo.DOUT_RIGHTLIGHT_RED,
                    RobotInfo.DOUT_RIGHTLIGHT_GREEN,
                    RobotInfo.DOUT_RIGHTLIGHT_BLUE);
        }
        else
        {
            leftLight = null;
            rightLight = null;
        }

        //
        // Vision subsystem.
        //
        if (visionTargetEnabled)
        {
            if (cameraServer != null)
            {
                visionTarget = new VisionTarget(this);
            }
        }
        else
        {
            visionTarget = null;
        }

        //
        // Robot Modes.
        //
        teleOpMode = new TeleOp(this);
        autoMode = new Autonomous(this);
        testMode = new Test(this);
        setupRobotModes(teleOpMode, autoMode, testMode, null);
    }   //initRobot

    public void setVideoEnabled(boolean enabled)
    {
        if (cameraServer != null)
        {
            if (enabled)
            {
                NIVision.IMAQdxStartAcquisition(usbCamSession);
            }
            else
            {
                NIVision.IMAQdxStopAcquisition(usbCamSession);
            }
            videoEnabled = enabled;
        }
    }   //setVideoEnabled

    public void updateDashboard()
    {
        double currTime = HalUtil.getCurrentTime();

        if (currTime >= nextUpdateTime)
        {
            nextUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;

            //
            // Sensor info.
            //
            /*
            sonarDistance = sonarFilter.filter(ultrasonic.getData());
            TrcDashboard.putNumber("Sonar Distance", sonarDistance);
            */

            //
            // USB camera streaming.
            //
            if (videoEnabled)
            {
                NIVision.IMAQdxGrab(usbCamSession, usbCamImage, 1);
                freshImage = true;
                NIVision.Rect rect = visionTarget != null? visionTarget.getLastTargetRect(): null;
                if (rect != null)
                {
//                    NIVision.imaqDuplicate(overlayImage, usbCamImage);
                    NIVision.imaqDrawShapeOnImage(
                            overlayImage,
                            usbCamImage,
//                            overlayImage,
                            rect,
                            DrawMode.DRAW_VALUE,
                            ShapeMode.SHAPE_RECT,
                            (float)0x0);
                    cameraServer.setImage(overlayImage);
                }
                else
                {
                    cameraServer.setImage(usbCamImage);
                }
            }

            if (debugDriveBase)
            {
                //
                // DriveBase debug info.
                //
                dashboard.displayPrintf(
                        1, "DriveBase: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f",
                        leftFrontMotor.getPosition(), rightFrontMotor.getPosition(),
                        leftRearMotor.getPosition(), rightRearMotor.getPosition());
                dashboard.displayPrintf(
                        2, "DriveBase: X=%.1f, Y=%.1f, Heading=%.1f",
                        driveBase.getXPosition()*RobotInfo.DRIVEBASE_X_SCALE,
                        driveBase.getYPosition()*RobotInfo.DRIVEBASE_Y_SCALE,
                        driveBase.getHeading());
            }

            if (debugArm)
            {
                arm.displayDebugInfo(3);
            }

            if (debugCrane)
            {
                crane.displayDebugInfo(7);
            }

            if (debugPidDrive)
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
        }
    }   //updateDashboard

    //
    // Implements TrcPidController.PidInput.
    //
    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == encoderXPidCtrl)
        {
            value = driveBase.getXPosition()*RobotInfo.DRIVEBASE_X_SCALE;
        }
        else if (pidCtrl == encoderYPidCtrl)
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

    //
    // Implements FrcVision.ImageProvider.
    //

    @Override
    public Image getImage()
    {
        return freshImage? usbCamImage: null;
    }   //getImage

}   //class Robot
