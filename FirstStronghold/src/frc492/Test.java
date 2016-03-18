package frc492;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class Test extends TeleOp
{
    public enum TestMode
    {
        SENSORS_TEST,
        DRIVEMOTORS_TEST,
        TIMED_DRIVE,
        X_DRIVE,
        Y_DRIVE,
        TURN,
        SONAR_DRIVE
    }   //enum TestMode

    private enum State
    {
        START,
        DONE
    }   //State

    private static final String moduleName = "Test";

    private HalDashboard dashboard = HalDashboard.getInstance();
    private TrcStateMachine sm;
    private TrcEvent event;
    private TrcTimer timer;
    private SendableChooser testChooser;
    private TestMode testMode = TestMode.SENSORS_TEST;
    private int motorIndex = 0;

    public Test(Robot robot)
    {
        super(robot);   // Call TeleOp constructor.

        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);

        testChooser = new SendableChooser();
        testChooser.addDefault("Sensors test", TestMode.SENSORS_TEST);
        testChooser.addObject("Drive motors test", TestMode.DRIVEMOTORS_TEST);
        testChooser.addObject("Drive for 8 sec", TestMode.TIMED_DRIVE);
        testChooser.addObject("Move X 20 ft", TestMode.X_DRIVE);
        testChooser.addObject("Move Y 20 ft", TestMode.Y_DRIVE);
        testChooser.addObject("Turn 360", TestMode.TURN);
        testChooser.addObject("Sonar drive 7 in", TestMode.SONAR_DRIVE);
        HalDashboard.putData("Robot tune modes", testChooser);
     }   //Autonomous

    //
    // Overriding TrcRobot.RobotMode.
    //

    @Override
    public void startMode()
    {
        super.startMode();  // Call TeleOp startMode.
        testMode = (TestMode)(testChooser.getSelected());

        switch (testMode)
        {
            case DRIVEMOTORS_TEST:
            case TIMED_DRIVE:
            case X_DRIVE:
            case Y_DRIVE:
            case SONAR_DRIVE:
                sm.start(State.START);
                break;

            default:
                break;
        }
    }   //startMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (testMode == TestMode.SENSORS_TEST)
        {
            super.runPeriodic(elapsedTime);
        }
//        LiveWindow.run();
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (testMode != TestMode.SENSORS_TEST)
        {
            dashboard.displayPrintf(1, "[%8.3f] %s", elapsedTime, sm.getState().toString());
        }

        switch (testMode)
        {
            case DRIVEMOTORS_TEST:
                doDriveMotorsTest();
                break;

            case TIMED_DRIVE:
                doTimedDrive();
                break;

            case X_DRIVE:
                doXDrive();
                break;

            case Y_DRIVE:
                doYDrive();
                break;

            case SONAR_DRIVE:
                doSonarDrive();
                break;

            default:
                break;
        }
    }   //runContinuous

    public void doDriveMotorsTest()
    {
        dashboard.displayPrintf(2, "Motors Test: index=%d", motorIndex);
        dashboard.displayPrintf(3, "Enc: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f",
                                robot.leftFrontMotor.getPosition(),
                                robot.rightFrontMotor.getPosition(),
                                robot.leftRearMotor.getPosition(),
                                robot.rightRearMotor.getPosition());

        if (sm.isReady())
        {
            State state = (State)sm.getState();

            switch (state)
            {
                case START:
                    //
                    // Spin a wheel for 5 seconds.
                    //
                    switch (motorIndex)
                    {
                        case 0:
                            robot.leftFrontMotor.setPower(0.5);
                            robot.rightFrontMotor.setPower(0.0);
                            robot.leftRearMotor.setPower(0.0);
                            robot.rightRearMotor.setPower(0.0);
                            break;

                        case 1:
                            robot.leftFrontMotor.setPower(0.0);
                            robot.rightFrontMotor.setPower(0.5);
                            robot.leftRearMotor.setPower(0.0);
                            robot.rightRearMotor.setPower(0.0);
                            break;

                        case 2:
                            robot.leftFrontMotor.setPower(0.0);
                            robot.rightFrontMotor.setPower(0.0);
                            robot.leftRearMotor.setPower(0.5);
                            robot.rightRearMotor.setPower(0.0);
                            break;

                        case 3:
                            robot.leftFrontMotor.setPower(0.0);
                            robot.rightFrontMotor.setPower(0.0);
                            robot.leftRearMotor.setPower(0.0);
                            robot.rightRearMotor.setPower(0.5);
                            break;
                    }
                    motorIndex = motorIndex + 1;
                    timer.set(5.0, event);
                    sm.addEvent(event);
                    sm.waitForEvents(motorIndex < 4? State.START: State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    }   //doDriveMotorsTest

    public void doTimedDrive()
    {
        double lfEnc = robot.leftFrontMotor.getPosition();
        double rfEnc = robot.rightFrontMotor.getPosition();
        double lrEnc = robot.leftRearMotor.getPosition();
        double rrEnc = robot.rightRearMotor.getPosition();
        dashboard.displayPrintf(2, "Timed Drive:");
        dashboard.displayPrintf(
                3, "Enc: lf=%.0f,rf=%.0f,lr=%.0f,rr=%.0f", lfEnc, rfEnc, lrEnc, rrEnc);
        dashboard.displayPrintf(3, "average=%f", (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
        dashboard.displayPrintf(4, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.driveBase.getXPosition(),
                                robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());

        if (sm.isReady())
        {
            State state = (State)sm.getState();

            switch (state)
            {
                case START:
                    robot.driveBase.mecanumDrive_Cartesian(0.0, 0.3, 0.0);
                    timer.set(8.0, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                    robot.driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    }   //doTimedDrive

    public void doXDrive()
    {
        dashboard.displayPrintf(2, "X Drive:");
        dashboard.displayPrintf(3, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.driveBase.getXPosition(),
                                robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());
        robot.encoderXPidCtrl.displayPidInfo(4);
        robot.encoderYPidCtrl.displayPidInfo(6);
        robot.gyroTurnPidCtrl.displayPidInfo(8);

        if (sm.isReady())
        {
            State state = (State)sm.getState();

            switch (state)
            {
                case START:
                    robot.pidDrive.setTarget(144.0, 0.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                    sm.stop();
                    break;
            }
        }
    }   //doXDrive

    public void doYDrive()
    {
        dashboard.displayPrintf(2, "Y Drive:");
        dashboard.displayPrintf(3, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.driveBase.getXPosition(),
                                robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());
        robot.encoderXPidCtrl.displayPidInfo(4);
        robot.encoderYPidCtrl.displayPidInfo(6);
        robot.gyroTurnPidCtrl.displayPidInfo(8);

        if (sm.isReady())
        {
            State state = (State)sm.getState();

            switch (state)
            {
                case START:
                    robot.pidDrive.setTarget(0.0, 144.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                    sm.stop();
                    break;
            }
        }
    }   //doYDrive

    public void doTurn()
    {
        dashboard.displayPrintf(2, "Turn:");
        dashboard.displayPrintf(3, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.driveBase.getXPosition(),
                                robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());
        robot.encoderXPidCtrl.displayPidInfo(4);
        robot.encoderYPidCtrl.displayPidInfo(6);
        robot.gyroTurnPidCtrl.displayPidInfo(8);

        if (sm.isReady())
        {
            State state = (State)sm.getState();

            switch (state)
            {
                case START:
                    robot.pidDrive.setTarget(0.0, 0.0, 360.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                    sm.stop();
                    break;
            }
        }
    }   //doTurn

    public void doSonarDrive()
    {
        dashboard.displayPrintf(2, "Sonar Drive:");
        dashboard.displayPrintf(3, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.driveBase.getXPosition(),
                                robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());
        robot.encoderXPidCtrl.displayPidInfo(4);
        robot.sonarYPidCtrl.displayPidInfo(6);
        robot.gyroTurnPidCtrl.displayPidInfo(8);

        if (sm.isReady())
        {
            State state = (State)sm.getState();

            switch (state)
            {
                case START:
                    robot.sonarPidDrive.setTarget(0.0, 8.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                    sm.stop();
                    break;
            }
        }
    }   //doSonarDrive

}   //class Test
