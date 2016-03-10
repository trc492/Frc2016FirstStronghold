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

    protected Robot robot;

    //
    // Input subsystem.
    //
    private FrcJoystick leftDriveStick;
    private FrcJoystick rightDriveStick;
    private FrcJoystick operatorStick;

    private TrcBooleanState ringLightPowerToggle = new TrcBooleanState("RingLightPower", false);
    private TrcBooleanState leftLightFlashingToggle =
            new TrcBooleanState("leftLightFlashing", false);
    private TrcBooleanState rightLightFlashingToggle =
            new TrcBooleanState("rightLightFlashing", false);
    private boolean slowDriveOverride = false;
    private boolean syncArmEnabled = true;
    private boolean sampleTilterAngle = false;
    private double prevTilterAngle = RobotInfo.TILTER_MIN_ANGLE;
    private int leftColorValue = 0;
    private int rightColorValue = 0;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;

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
        robot.crane.zeroCalibrateTilter();
        robot.crane.zeroCalibarateCrane();
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
        robot.arm.setPower(power, syncArmEnabled);

        if (sampleTilterAngle)
        {
            double angle = (operatorStick.getZ() + 1.0)/2.0*
                           (RobotInfo.TILTER_MAX_ANGLE - RobotInfo.TILTER_MIN_ANGLE) +
                           RobotInfo.TILTER_MIN_ANGLE;
            if (angle != prevTilterAngle)
            {
                prevTilterAngle = angle;
                robot.crane.setTilterAngle(angle);
            }
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

                case FrcJoystick.LOGITECH_BUTTON6:
                    if (robot.leftLight != null)
                    {
                        if (pressed)
                        {
                            leftLightFlashingToggle.toggleState();
                            if (leftLightFlashingToggle.getState())
                            {
                                robot.leftLight.setColor(
                                        RGBColor.getColor(leftColorValue), 0.5, 0.5);
                            }
                            else
                            {
                                robot.leftLight.setColor(RGBColor.getColor(leftColorValue));
                            }
                        }
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON7:
                    if (robot.leftLight != null)
                    {
                        if (pressed)
                        {
                            leftColorValue++;
                            if (leftColorValue > TrcRGBLight.COLOR_MAX_VALUE)
                            {
                                leftColorValue = TrcRGBLight.COLOR_MIN_VALUE;
                            }

                            if (leftLightFlashingToggle.getState())
                            {
                                robot.leftLight.setColor(
                                        RGBColor.getColor(leftColorValue), 0.5, 0.5);
                            }
                            else
                            {
                                robot.leftLight.setColor(RGBColor.getColor(leftColorValue));
                            }
                        }
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON8:
                    if (debugVision)
                    {
                        if (pressed && robot.visionTarget != null)
                        {
                            robot.visionTarget.setRingLightPowerOn(
                                    ringLightPowerToggle.toggleState());
                        }
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON9:
                    if (debugVision)
                    {
                        if (pressed && robot.visionTarget != null)
                        {
                            robot.visionTarget.getTargetReport();
                        }
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON10:
                    if (robot.rightLight != null)
                    {
                        if (pressed)
                        {
                            rightColorValue++;
                            if (rightColorValue > TrcRGBLight.COLOR_MAX_VALUE)
                            {
                                rightColorValue = TrcRGBLight.COLOR_MIN_VALUE;
                            }

                            if (rightLightFlashingToggle.getState())
                            {
                                robot.rightLight.setColor(
                                        RGBColor.getColor(rightColorValue), 0.5, 0.5);
                            }
                            else
                            {
                                robot.rightLight.setColor(RGBColor.getColor(rightColorValue));
                            }
                        }
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON11:
                    if (robot.rightLight != null)
                    {
                        if (pressed)
                        {
                            rightLightFlashingToggle.toggleState();
                            if (rightLightFlashingToggle.getState())
                            {
                                robot.rightLight.setColor(
                                        RGBColor.getColor(rightColorValue), 0.5, 0.5);
                            }
                            else
                            {
                                robot.rightLight.setColor(RGBColor.getColor(rightColorValue));
                            }
                        }
                    }
                    break;
            }
        }
        else if (joystick == rightDriveStick)
        {
            switch (button)
            {
                case FrcJoystick.SIDEWINDER_TRIGGER:
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
                        robot.crane.setTilterAngle(70.0);
                        robot.crane.setCraneLength(RobotInfo.CRANE_MAX_LENGTH);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON5:
                    break;

                case FrcJoystick.LOGITECH_BUTTON6:
                    if (pressed)
                    {
                        robot.crane.setCranePower(RobotInfo.CRANE_POWER);
                    }
                    else
                    {
                        robot.crane.setCranePower(0.0);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON7:
                    if (pressed)
                    {
                        robot.crane.setCranePower(-RobotInfo.CRANE_POWER);
                    }
                    else
                    {
                        robot.crane.setCranePower(0.0);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON8:
                    sampleTilterAngle = pressed;
                    break;

                case FrcJoystick.LOGITECH_BUTTON9:
                    if (pressed)
                    {
                        robot.arm.zeroCalibrate();
                        robot.crane.zeroCalibrateTilter();
                        robot.crane.zeroCalibarateCrane();
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON10:
                    if (pressed)
                    {
                        robot.crane.setWinchPower(RobotInfo.WINCH_POWER);
                    }
                    else
                    {
                        robot.crane.setWinchPower(0.0);
                    }
                    break;

                case FrcJoystick.LOGITECH_BUTTON11:
                    if (pressed)
                    {
                        robot.crane.setWinchPower(-RobotInfo.WINCH_POWER);
                    }
                    else
                    {
                        robot.crane.setWinchPower(0.0);
                    }
                    break;
            }
        }
    }   //joystickButtonEvent

}   //class TeleOp
