package frc492;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import frclib.FrcJoystick;
import frclib.FrcRGBLight;
import frclib.FrcRobotBase;
import trclib.TrcBooleanState;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;

public class TeleOp implements TrcRobot.RobotMode, FrcJoystick.ButtonHandler
{
    private static final boolean debugElevator = false;
    private static final boolean debugVision = false;

    private static final FrcRGBLight.RGBColor[] colorTable =
    {
        FrcRGBLight.RGBColor.RGB_BLACK,
        FrcRGBLight.RGBColor.RGB_RED,
        FrcRGBLight.RGBColor.RGB_GREEN,
        FrcRGBLight.RGBColor.RGB_YELLOW,
        FrcRGBLight.RGBColor.RGB_BLUE,
        FrcRGBLight.RGBColor.RGB_MAGENTA,
        FrcRGBLight.RGBColor.RGB_CYAN,
        FrcRGBLight.RGBColor.RGB_WHITE
    };

    private Robot robot;
    //
    // Input subsystem.
    //
    private FrcJoystick leftDriveStick;
    private FrcJoystick rightDriveStick;
    private FrcJoystick operatorStick;

    private TrcBooleanState ringLightPowerToggle;
    private TrcBooleanState ledFlashingToggle;
    private boolean slowDriveOverride;
    private boolean slowElevatorOverride;
    private int colorIndex;

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

        ringLightPowerToggle = new TrcBooleanState("ringLightPower", false);
        ledFlashingToggle = new TrcBooleanState("LEDFlashing", false);

        slowDriveOverride = false;
        slowElevatorOverride = false;
        colorIndex = 0;
    }   //TeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //
    
    public void startMode()
    {
//        robot.elevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
//        robot.ultrasonic.setEnabled(true);
        robot.driveBase.resetPosition();
        if (debugVision)
        {
            if (robot.visionTarget != null)
            {
                robot.visionTarget.setVisionTaskEnabled(true);
            }
        }
    }   //startMode

    public void stopMode()
    {
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
        TrcDbgTrace dbgTrace = FrcRobotBase.getRobotTracer();
        
        /*
        double p = rightDriveStick.getYWithDeadband(true);
        robot.leftFrontMotor.setPower(p);
        */
        /*
        dbgTrace.traceInfo(
                "TeleOpPeriodic", "Enc: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f, %s",
                robot.leftFrontMotor.getPosition(), robot.rightFrontMotor.getPosition(),
                robot.leftRearMotor.getPosition(), robot.rightRearMotor.getPosition(),
                robot.leftFrontMotor.isSensorPresent(FeedbackDevice.QuadEncoder).toString());
                */
        //
        // DriveBase operation.
        //
        double x = leftDriveStick.getXWithDeadband(true);
        double y = rightDriveStick.getYWithDeadband(true);
        double rot = rightDriveStick.getTwistWithDeadband(true);
        robot.driveBase.mecanumDrive_Cartesian(x, y, rot);
        /*
        double leftPower = leftDriveStick.getYWithDeadband(true);
        double rightPower = rightDriveStick.getYWithDeadband(true);
        robot.driveBase.tankDrive(leftPower, rightPower);
        */
        /*
        double drivePower = rightDriveStick.getYWithDeadband(true);
        double turnPower = rightDriveStick.getTwistWithDeadband(true);
        if (slowDriveOverride)
        {
            drivePower /= RobotInfo.DRIVE_SLOW_YSCALE;
            turnPower /= RobotInfo.DRIVE_SLOW_TURNSCALE;
        }
        robot.driveBase.arcadeDrive(drivePower, turnPower);
        */

        //
        // Elevator operation.
        //
        double elevatorPower = operatorStick.getYWithDeadband(true);
        if (slowElevatorOverride)
        {
            elevatorPower /= 2.0;
        }
        
        robot.elevator.setPower(elevatorPower);

        if (debugElevator)
        {
            robot.elevator.displayDebugInfo(1);
        }
        /*
        double power = operatorStick.getYWithDeadband(true);
        robot.arm.setPower(power);
        */

        robot.updateDashboard();
    }   //runPeriodic

    public void runContinuous(double elapsedTime)
    {
    }   //runContinuous

    //
    // Implements TrcJoystick.ButtonHandler.
    //

    public void joystickButtonEvent(
            FrcJoystick joystick,
            int buttonMask,
            boolean pressed)
    {
        if (joystick == leftDriveStick)
        {
            switch (buttonMask)
            {
            case FrcJoystick.LOGITECH_TRIGGER:
                slowDriveOverride = pressed;
                break;

            case FrcJoystick.LOGITECH_BUTTON8:
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

            case FrcJoystick.LOGITECH_BUTTON9:
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

            case FrcJoystick.LOGITECH_BUTTON10:
                if (debugVision)
                {
                    if (pressed)
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
            switch (buttonMask)
            {
            case FrcJoystick.LOGITECH_TRIGGER:
                break;
            }
        }
        else if (joystick == operatorStick)
        {
            switch (buttonMask)
            {
            case FrcJoystick.LOGITECH_TRIGGER:
                robot.elevator.setElevatorOverride(pressed);
                break;

            case FrcJoystick.LOGITECH_BUTTON2:
                slowElevatorOverride = pressed;
                break;

            case FrcJoystick.LOGITECH_BUTTON4:
                if (pressed)
                {
                    robot.arm.setPower(0.5);
                }
                else
                {
                    robot.arm.setPower(0.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON5:
                if (pressed)
                {
                    robot.arm.setPower(-0.5);
                }
                else
                {
                    robot.arm.setPower(0.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON6:
                if (pressed)
                {
                    robot.elevator.resetPosition();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON7:
                if (pressed)
                {
                    robot.elevator.zeroCalibrate();
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON10:
                if (pressed)
                {
                    robot.pickup.set(0.5);
                }
                else
                {
                    robot.pickup.set(0.0);
                }
                break;

            case FrcJoystick.LOGITECH_BUTTON11:
                if (pressed)
                {
                    robot.pickup.set(-1.0);
                }
                else
                {
                    robot.pickup.set(0.0);
                }
                break;
            }
        }
    }   //joystickButtonEvent

}   //class TeleOp
