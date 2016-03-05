package frc492;

import frclib.FrcJoystick;
import frclib.FrcRobotBase;
import trclib.TrcBooleanState;
import trclib.TrcDbgTrace;
import trclib.TrcRGBLight;
import trclib.TrcRobot;
import trclib.TrcRGBLight.RGBColor;

public class TeleOp implements TrcRobot.RobotMode, FrcJoystick.ButtonHandler
{
    protected TrcDbgTrace dbgTrace = FrcRobotBase.getRobotTracer();
    private static final boolean debugVision = true;

    private enum DriveMode
    {
        MECANUM_MODE,
        ARCADE_MODE,
        TANK_MODE
    }   //enum DriveMode

    private enum OperatorStickMode
    {
        ARM_MODE,
        WINCH_MODE
    }   //enum OperatorStickMode

    protected Robot robot;

    //
    // Input subsystem.
    //
    private FrcJoystick leftDriveStick;
    private FrcJoystick rightDriveStick;
    private FrcJoystick operatorStick;

    private TrcBooleanState ringLightPowerToggle = new TrcBooleanState("RingLightPower", false);
    private TrcBooleanState ledFlashingToggle = new TrcBooleanState("LEDFlashing", false);
    private boolean slowDriveOverride = false;
    private boolean syncArmEnabled = true;
    private int colorValue = 0;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;
    private OperatorStickMode operatorStickMode = OperatorStickMode.ARM_MODE;

    public TeleOp(Robot robot)
    {
        this.robot = robot;
        //
        // Input subsystem.
        //
        leftDriveStick = new FrcJoystick(
                "leftDriveStick",
                RobotInfo.JSPORT_LEFT_DRIVESTICK, this);
        leftDriveStick.setYInverted(true);

        rightDriveStick = new FrcJoystick(
                "rightDriveStick",
                RobotInfo.JSPORT_RIGHT_DRIVESTICK, this);
        rightDriveStick.setYInverted(true);

        operatorStick = new FrcJoystick(
                "operatorStick",
                RobotInfo.JSPORT_OPERATORSTICK, this);
        operatorStick.setYInverted(true);
    }   //TeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    public void startMode()
    {
        robot.arm.zeroCalibrate();
//        robot.ultrasonic.setEnabled(true);
        robot.driveBase.resetPosition();

        if (debugVision)
        {
            if (robot.visionTarget != null)
            {
                robot.visionTarget.setVisionTaskEnabled(true);
            }
        }
        robot.setVideoEnabled(true);
    }   //startMode

    public void stopMode()
    {
        robot.setVideoEnabled(false);
        robot.driveBase.stop();
//        robot.ultrasonic.setEnabled(false);

        if (debugVision)
        {
            if (robot.visionTarget != null)
            {
                robot.visionTarget.setVisionTaskEnabled(false);
            }
        }
    }   //stopMode

    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase operation.
        //
        switch (driveMode)
        {
            case TANK_MODE:
                double leftPower = leftDriveStick.getYWithDeadband(true);
                double rightPower = rightDriveStick.getYWithDeadband(true);
                if (slowDriveOverride)
                {
                    leftPower /= RobotInfo.DRIVE_SLOW_YSCALE;
                    rightPower /= RobotInfo.DRIVE_SLOW_YSCALE;
                }
                robot.driveBase.tankDrive(leftPower, rightPower);
                break;

            case ARCADE_MODE:
                double drivePower = rightDriveStick.getYWithDeadband(true);
                double turnPower = rightDriveStick.getTwistWithDeadband(true);
                if (slowDriveOverride)
                {
                    drivePower /= RobotInfo.DRIVE_SLOW_YSCALE;
                    turnPower /= RobotInfo.DRIVE_SLOW_TURNSCALE;
                }
                robot.driveBase.arcadeDrive(drivePower, turnPower);
                break;

            default:
            case MECANUM_MODE:
                double x = leftDriveStick.getXWithDeadband(true);
                double y = rightDriveStick.getYWithDeadband(true);
                double rot = rightDriveStick.getTwistWithDeadband(true);
                if (slowDriveOverride)
                {
                    x /= RobotInfo.DRIVE_SLOW_XSCALE;
                    y /= RobotInfo.DRIVE_SLOW_YSCALE;
                    rot /= RobotInfo.DRIVE_SLOW_TURNSCALE;
                }
                robot.driveBase.mecanumDrive_Cartesian(x, y, rot);
                break;
        }

        //
        // Arm/Winch operation.
        //
        double power = operatorStick.getYWithDeadband(true);
        if (operatorStickMode == OperatorStickMode.ARM_MODE)
        {
            robot.arm.setPower(power, syncArmEnabled);
        }
        else
        {
            robot.crane.setWinchPower(power);
        }

        robot.updateDashboard();
    }   //runPeriodic

    public void runContinuous(double elapsedTime)
    {
    }   //runContinuous

    //
    // Implements TrcJoystick.ButtonHandler.
    //

    public void joystickButtonEvent(FrcJoystick joystick, int button, boolean pressed)
    {
        if (joystick == leftDriveStick)
        {
            switch (button)
            {
                case FrcJoystick.LOGITECH_TRIGGER:
                    break;
    
                case FrcJoystick.LOGITECH_BUTTON3:
                    if (pressed)
                    {
                        driveMode = DriveMode.MECANUM_MODE;
                    }
                    break;
                    
                case FrcJoystick.LOGITECH_BUTTON4:
                    if (pressed)
                    {
                        driveMode = DriveMode.TANK_MODE;
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON5:
                    if (pressed)
                    {
                        driveMode = DriveMode.ARCADE_MODE;
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON8:
                    if (robot.rgbLight != null)
                    {
                        if (pressed)
                        {
                            ledFlashingToggle.toggleState();
                            if (ledFlashingToggle.getState())
                            {
                                robot.rgbLight.setColor(RGBColor.getColor(colorValue), 0.5, 0.5);
                            }
                            else
                            {
                                robot.rgbLight.setColor(RGBColor.getColor(colorValue));
                            }
                        }
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON9:
                    if (robot.rgbLight != null)
                    {
                        if (pressed)
                        {
                            colorValue++;
                            if (colorValue > TrcRGBLight.COLOR_MAX_VALUE)
                            {
                                colorValue = TrcRGBLight.COLOR_MIN_VALUE;
                            }

                            if (ledFlashingToggle.getState())
                            {
                                robot.rgbLight.setColor(RGBColor.getColor(colorValue), 0.5, 0.5);
                            }
                            else
                            {
                                robot.rgbLight.setColor(RGBColor.getColor(colorValue));
                            }
                        }
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON10:
                    if (debugVision)
                    {
                        if (pressed && robot.visionTarget != null)
                        {
                            robot.visionTarget.getTargetReport();
                        }
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON11:
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
            switch (button)
            {
                case FrcJoystick.LOGITECH_TRIGGER:
                    slowDriveOverride = pressed;
                    break;
            }
        }
        else if (joystick == operatorStick)
        {
            switch (button)
            {
                case FrcJoystick.LOGITECH_TRIGGER:
                    syncArmEnabled = !pressed;
                    break;

                case FrcJoystick.LOGITECH_BUTTON2:
                    if (pressed)
                    {
                        robot.pickup.set(RobotInfo.PICKUP_IN_POWER);
                    }
                    else
                    {
                        robot.pickup.set(0.0);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON3:
                    if (pressed)
                    {
                        robot.pickup.set(-RobotInfo.PICKUP_OUT_POWER);
                    }
                    else
                    {
                        robot.pickup.set(0.0);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON4:
                    if (pressed)
                    {
                        operatorStickMode = OperatorStickMode.ARM_MODE;
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON5:
                    if (pressed)
                    {
                        operatorStickMode = OperatorStickMode.WINCH_MODE;
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON6:
                    if (pressed)
                    {
                        robot.crane.setCranePower(-RobotInfo.CRANE_POWER);
                    }
                    else
                    {
                        robot.crane.setCranePower(0.0);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON7:
                    if (pressed)
                    {
                        robot.crane.setCranePower(RobotInfo.CRANE_POWER);
                    }
                    else
                    {
                        robot.crane.setCranePower(0.0);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON8:
                    if (pressed)
                    {
                        robot.arm.zeroCalibrate();
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON9:
                    if (pressed)
                    {
                        robot.crane.zeroCalibrateTilter();
                        robot.crane.zeroCalibarateCrane();
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON10:
                    if (pressed)
                    {
                        robot.crane.setTilterPower(-RobotInfo.TILTER_POWER);
                    }
                    else
                    {
                        robot.crane.setTilterPower(0.0);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON11:
                    if (pressed)
                    {
                        robot.crane.setTilterPower(RobotInfo.TILTER_POWER);
                    }
                    else
                    {
                        robot.crane.setTilterPower(0.0);
                    }
                    break;
            }
        }
    }   //joystickButtonEvent

}   //class TeleOp
