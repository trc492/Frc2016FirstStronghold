package frc492;

import edu.wpi.first.wpilibj.Timer;
import frclibj.TrcAnalogInput;
import frclibj.TrcAnalogInput.Zone;
import frclibj.TrcDashboard;
import frclibj.TrcJoystick;
import frclibj.TrcRGBLight;
import frclibj.TrcRobot;
import frclibj.TrcBooleanState;

public class TeleOp implements TrcRobot.RobotMode,
                               TrcJoystick.ButtonHandler,
                               TrcAnalogInput.AnalogEventHandler
{
    private static final boolean debugElevator = false;
    private static final boolean debugVision = false;
    private static final String grabberToggleEnabledKey =
            "Grabber Toggle Enabled";
    private static final String pusherToggleEnabledKey =
            "Pusher Toggle Enabled";
    private static final String presetElevatorHeightKey =
            "Preset Elevator Height";

    private static final TrcRGBLight.RGBColor[] colorTable =
    {
        TrcRGBLight.RGBColor.RGB_BLACK,
        TrcRGBLight.RGBColor.RGB_RED,
        TrcRGBLight.RGBColor.RGB_GREEN,
        TrcRGBLight.RGBColor.RGB_YELLOW,
        TrcRGBLight.RGBColor.RGB_BLUE,
        TrcRGBLight.RGBColor.RGB_MAGENTA,
        TrcRGBLight.RGBColor.RGB_CYAN,
        TrcRGBLight.RGBColor.RGB_WHITE
    };

    private Robot robot;
    //
    // Input subsystem.
    //
    private TrcJoystick leftDriveStick;
    private TrcJoystick rightDriveStick;
    private TrcJoystick operatorStick;

    private TrcBooleanState ringLightPowerToggle;
    private TrcBooleanState ledFlashingToggle;
    private boolean slowDriveOverride;
    private boolean slowElevatorOverride;
    private boolean headingLockEnabled;
    private int colorIndex;
    private TrcRGBLight.ColorLightState savedLightState;
    private boolean grabberToggleEnabled = true;
    private boolean pusherToggleEnabled = true;
    private boolean sonarAlignmentOn = false;
    private double presetElevatorHeight = 3.0;

    private double prevRobotTilt;
    private TrcBooleanState robotToppling;
    private TrcRGBLight.ColorLightState toppleLightState;
    private double topplingResetTime;
    private double driveResetTime;
    private double topplingTilt;

    public TeleOp(Robot robot)
    {
        this.robot = robot;
        //
        // Input subsystem.
        //
        leftDriveStick = new TrcJoystick(
                "LeftDriveStick",
                RobotInfo.JSPORT_LEFT_DRIVESTICK, this);
        rightDriveStick = new TrcJoystick(
                "RightDriveStick",
                RobotInfo.JSPORT_RIGHT_DRIVESTICK, this);
        rightDriveStick.setYInverted(true);
        operatorStick = new TrcJoystick(
                "OperatorStick",
                RobotInfo.JSPORT_OPERATORSTICK, this);

        ringLightPowerToggle = new TrcBooleanState("ringLightPower", false);
        ledFlashingToggle = new TrcBooleanState("LEDFlashing", false);

        slowDriveOverride = false;
        slowElevatorOverride = false;
        headingLockEnabled = false;
        colorIndex = 0;
        savedLightState = null;

        grabberToggleEnabled = TrcDashboard.getBoolean(
                grabberToggleEnabledKey, grabberToggleEnabled);
        pusherToggleEnabled = TrcDashboard.getBoolean(
                pusherToggleEnabledKey, pusherToggleEnabled);

        prevRobotTilt = robot.robotTilt;
        robotToppling = new TrcBooleanState("robotToppling", false);
        toppleLightState = null;
        topplingResetTime = 0.0;
        driveResetTime = 0.0;
        topplingTilt = 0.0;
        //
        // Use coast mode to prevent the tote/bin stack from toppling over.
        //
        robot.driveBase.setBrakeModeEnabled(false);
    }   //TeleOp

    //
    // Implements TrcRobot.RunMode.
    //
    public void start()
    {
        robot.sonarPidCtrl.setOutputRange(
                -RobotInfo.SONAR_RANGE_LIMIT, RobotInfo.SONAR_RANGE_LIMIT);
        robot.leftElevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
        robot.rightElevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
        robot.ultrasonic.setEnabled(true);
        if (debugVision)
        {
            if (robot.visionTarget != null)
            {
                robot.visionTarget.setVisionTaskEnabled(true);
            }
        }
    }   //start

    public void stop()
    {
        robot.driveBase.stop();
        robot.ultrasonic.setEnabled(false);
        if (debugVision)
        {
            if (robot.visionTarget != null)
            {
                robot.visionTarget.setVisionTaskEnabled(false);
            }
        }
    }   //stop

    public void periodic()
    {
//robot.sonarPidCtrl.setKp(TrcDashboard.getNumber("SonarKp", RobotInfo.SONAR_KP));
        presetElevatorHeight = TrcDashboard.getNumber(
                presetElevatorHeightKey, presetElevatorHeight);
        //
        // Robot toppling detection.
        //
        double elevatorPower = operatorStick.getYWithDeadband(true);
        if (robotToppling.getState())
        {
            topplingTilt = robot.robotTilt;
            double currTime = Timer.getFPGATimestamp();
            if (currTime >= driveResetTime)
            {
//                robot.driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
            }

            if (currTime >= topplingResetTime)
            {
                //
                // Clear toppling state.
                //
                robotToppling.toggleState();
                if (robot.rgbLight != null)
                {
                    robot.rgbLight.setColorLightState(toppleLightState);
                }
                topplingTilt = 0.0;
            }
        }
        else if (robot.robotTilt <= RobotInfo.ROBOT_TOPPLE_THRESHOLD &&
                 robot.robotTilt < prevRobotTilt &&
                 elevatorPower < 0.0 &&
                 (Math.abs(robot.driveBase.getYSpeed()) <=
                  RobotInfo.ROBOT_YSPEED_THRESHOLD))
        {
            //
            // The robot is about to topple over.
            //
            double currTime = Timer.getFPGATimestamp();
            robotToppling.toggleState();
            topplingResetTime = currTime + RobotInfo.ROBOT_TOPPLE_RESET_TIME;
            driveResetTime = currTime + RobotInfo.ROBOT_DRIVE_RESET_TIME;
            if (robot.rgbLight != null)
            {
                toppleLightState = robot.rgbLight.getColorLightState();
                robot.rgbLight.setColor(TrcRGBLight.RGBColor.RGB_MAGENTA);
            }
            //
            // Do a quick step backward.
            //
//            robot.driveBase.mecanumDrive_Cartesian(0.0, -0.5, 0.0, 0.0);
            //
            // Raise elevator 3 inches
            //
//            robot.leftElevator.setHeight(robot.leftElevator.getHeight() + 3.0);
        }
        prevRobotTilt = robot.robotTilt;
        TrcDashboard.putNumber("Toppling Tilt", topplingTilt);

        //
        // DriveBase operation.
        //
//        if (!robotToppling.getState())
        if (!sonarAlignmentOn)
        {
            double x = leftDriveStick.getXWithDeadband(true);
            double y = rightDriveStick.getYWithDeadband(true);
            double rot = rightDriveStick.getTwistWithDeadband(true);
            if (slowDriveOverride)
            {
                x /= RobotInfo.DRIVE_SLOW_XSCALE;
                y /= RobotInfo.DRIVE_SLOW_YSCALE;
                rot /= RobotInfo.DRIVE_SLOW_TURNSCALE;
            }

            robot.driveBase.mecanumDrive_Cartesian(
                    x, y, rot,
                    headingLockEnabled? robot.gyro.getAngle(): 0.0);
        }
        //
        // Elevator operation.
        //
        if (slowElevatorOverride)
        {
            elevatorPower /= 2.0;
        }
        robot.leftElevator.setPower(elevatorPower);
        robot.rightElevator.setPower(elevatorPower);
        /*
        if (!robotToppling.getState())
        {
            robot.leftElevator.setPower(elevatorPower);
        }
        */

        if (debugElevator)
        {
            robot.leftElevator.displayDebugInfo(1);
            robot.rightElevator.displayDebugInfo(1);
        }
        robot.updateDashboard();
    }   //periodic

    public void continuous()
    {
    }   //continuous

    //
    // Implements TrcJoystick.ButtonHandler.
    //
    public void joystickButtonEvent(
            TrcJoystick joystick,
            int buttonMask,
            boolean pressed)
    {
//        System.out.printf("stick=%s, button=%x, pressed=%s\n",
//                joystick.toString(), buttonMask, Boolean.toString(pressed));
        if (joystick == leftDriveStick)
        {
            switch (buttonMask)
            {
            case TrcJoystick.LOGITECH_TRIGGER:
                slowDriveOverride = pressed;
                break;

            case TrcJoystick.LOGITECH_BUTTON7:
                if (pressed)
                {
                    robot.sonarPidDrive.setTarget(
                            0.0, robot.dispenserDistance, 0.0,
                            false, null, 0.0);
                }
                else if (robot.sonarPidDrive.isEnabled())
                {
                    robot.sonarPidDrive.cancel();
                }
                robot.driveBase.setBrakeModeEnabled(pressed);
                sonarAlignmentOn = pressed;
                break;

            case TrcJoystick.LOGITECH_BUTTON8:
                if (robot.rgbLight != null)
                {
                    if (pressed)
                    {
                        ledFlashingToggle.toggleState();
                        if (ledFlashingToggle.getState())
                        {
                            robot.rgbLight.setColor(
                                    colorTable[colorIndex], 0.5, 0.5, null);
                        }
                        else
                        {
                            robot.rgbLight.setColor(colorTable[colorIndex]);
                        }
                    }
                }
                break;

            case TrcJoystick.LOGITECH_BUTTON9:
                if (robot.rgbLight != null)
                {
                    if (pressed)
                    {
                        colorIndex++;
                        if (colorIndex >= colorTable.length)
                        {
                            colorIndex = 0;
                        }

                        if (ledFlashingToggle.getState())
                        {
                            robot.rgbLight.setColor(
                                    colorTable[colorIndex], 0.5, 0.5, null);
                        }
                        else
                        {
                            robot.rgbLight.setColor(colorTable[colorIndex]);
                        }
                    }
                }
                break;

            case TrcJoystick.LOGITECH_BUTTON10:
                if (debugVision)
                {
                    if (pressed)
                    {
                        robot.visionTarget.getTargetReport();
                    }
                }
                break;

            case TrcJoystick.LOGITECH_BUTTON11:
                if (debugVision)
                {
                    if (pressed && robot.visionTarget != null)
                    {
                        robot.visionTarget.setRingLightPowerOn(
                                ringLightPowerToggle.toggleState());
                    }
                }
                break;
            }
        }
        else if (joystick == rightDriveStick)
        {
            switch (buttonMask)
            {
            case TrcJoystick.LOGITECH_TRIGGER:
                headingLockEnabled = pressed;
                if (pressed)
                {
                    robot.gyro.reset();
                }
                break;
            }
        }
        else if (joystick == operatorStick)
        {
            grabberToggleEnabled = TrcDashboard.getBoolean(
                    grabberToggleEnabledKey, grabberToggleEnabled);
            pusherToggleEnabled = TrcDashboard.getBoolean(
                    pusherToggleEnabledKey, pusherToggleEnabled);
            switch (buttonMask)
            {
            case TrcJoystick.LOGITECH_TRIGGER:
                robot.leftElevator.setElevatorOverride(pressed);
                robot.rightElevator.setElevatorOverride(pressed);
                break;

            case TrcJoystick.LOGITECH_BUTTON2:
                slowElevatorOverride = pressed;
                break;

            case TrcJoystick.LOGITECH_BUTTON3:
                if (robot.pusher != null)
                {
                    robot.pusher.setState(pressed, pusherToggleEnabled);
                }
                break;

            case TrcJoystick.LOGITECH_BUTTON4:
                if (robot.upperGrabber != null)
                {
                    robot.upperGrabber.setState(pressed, grabberToggleEnabled);
                    if (pressed && grabberToggleEnabled &&
                        robot.lowerGrabber != null &&
                        robot.rgbLight != null)
                    {
                        if (robot.upperGrabber.isExtended())
                        {
                            if (!robot.lowerGrabber.isExtended())
                            {
                                savedLightState =
                                        robot.rgbLight.getColorLightState();
                            }
                            robot.rgbLight.setColor(
                                    TrcRGBLight.RGBColor.RGB_GREEN);
                        }
                        else
                        {
                            if (robot.lowerGrabber.isExtended())
                            {
                                robot.rgbLight.setColor(
                                        TrcRGBLight.RGBColor.RGB_RED);
                            }
                            else
                            {
                                robot.rgbLight.setColorLightState(
                                        savedLightState);
                            }
                        }
                    }
                }
                //System.out.printf("Firing...\n");
                //robot.leftHook.setState(pressed, true);
                //robot.rightHook.setState(pressed, true);
                break;

            case TrcJoystick.LOGITECH_BUTTON5:
                if (robot.lowerGrabber != null)
                {
                    robot.lowerGrabber.setState(pressed, grabberToggleEnabled);
                    if (pressed && grabberToggleEnabled &&
                        robot.upperGrabber != null &&
                        robot.rgbLight != null)
                    {
                        if (robot.lowerGrabber.isExtended())
                        {
                            if (!robot.upperGrabber.isExtended())
                            {
                                savedLightState =
                                        robot.rgbLight.getColorLightState();
                            }
                            robot.rgbLight.setColor(
                                    TrcRGBLight.RGBColor.RGB_RED);
                        }
                        else
                        {
                            if (robot.upperGrabber.isExtended())
                            {
                                robot.rgbLight.setColor(
                                        TrcRGBLight.RGBColor.RGB_GREEN);
                            }
                            else
                            {
                                robot.rgbLight.setColorLightState(
                                        savedLightState);
                            }
                        }
                    }
                }
                break;

            case TrcJoystick.LOGITECH_BUTTON6:
                break;

            case TrcJoystick.LOGITECH_BUTTON7:
                robot.leftElevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
                robot.rightElevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
                break;

            case TrcJoystick.LOGITECH_BUTTON8:
                robot.leftElevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
                robot.rightElevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
                break;

            case TrcJoystick.LOGITECH_BUTTON9:
                if (pressed)
                {
                    robot.leftElevator.setHeight(presetElevatorHeight);
                    robot.rightElevator.setHeight(presetElevatorHeight);
                }
                break;

            case TrcJoystick.LOGITECH_BUTTON10:
                if (pressed)
                {
                    robot.leftElevator.setDeltaHeight(
                            -RobotInfo.ELEVATOR_HEIGHT_INC);
                    robot.rightElevator.setDeltaHeight(
                            -RobotInfo.ELEVATOR_HEIGHT_INC);
                }
                break;

            case TrcJoystick.LOGITECH_BUTTON11:
                if (pressed)
                {
                    robot.leftElevator.setDeltaHeight(
                            RobotInfo.ELEVATOR_HEIGHT_INC);
                    robot.rightElevator.setDeltaHeight(
                            -RobotInfo.ELEVATOR_HEIGHT_INC);
                }
                break;
            }
        }
    }   //joystickButtonEvent

    //
    // Implements TrcAnalogInput.AnalogEventHandler.
    //
    public void AnalogEvent(
            TrcAnalogInput analogInput,
            Zone zone,
            double value)
    {
        //
        // Check sonar distance. If within dispenserDistance, flash LED
        // with whatever the current color is. Otherwise flash CYAN.
        // If sonar distance went outside dispenserDistance, make the LED
        // color solid again, or if the color was CYAN, turn LED back off.
        //
        if (analogInput == robot.ultrasonic && robot.rgbLight != null)
        {
            TrcRGBLight.RGBColor color;
            switch (zone)
            {
            case ANALOGINPUT_MID_ZONE:
                color = robot.rgbLight.getColor();
                if (color == TrcRGBLight.RGBColor.RGB_BLACK)
                {
                    color = TrcRGBLight.RGBColor.RGB_CYAN;
                }
                robot.rgbLight.setColor(color, 0.1, 0.1, null);
                break;

            default:
                color = robot.rgbLight.getColor();
                if (color == TrcRGBLight.RGBColor.RGB_CYAN)
                {
                    color = TrcRGBLight.RGBColor.RGB_BLACK;
                }
                robot.rgbLight.setColor(color);
                break;
            }
        }
    }   //AnalogEvent

}   //class TeleOp
