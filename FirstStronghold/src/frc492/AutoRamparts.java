package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

/*
Autonomous for ramparts
rebecca cheng~~
 */

public class AutoRamparts implements AutoStrategy
{
    // variables
    private static final String moduleName = "AutoRamparts";
    private HalDashboard dashboard = HalDashboard.getInstance();

    private Robot robot;
    private double distanceToDefense;
    private double distanceApproachDefense;
    private double distanceCrossDefense;
    private TrcStateMachine sm;
    private TrcEvent event;
    private TrcTimer timer;

    // state machine
    private enum State
    {
        DRIVE_TO_RAMPARTS,
        APPROACH_RAMPARTS,
        LOWER_ARMS,
        OVER_RAMPARTS,
        RAISE_ARMS,
        DONE
    }

    // constructor takes: robot, distance from neutral line
    public AutoRamparts (Robot robot)
    {
        this.robot = robot;
        distanceToDefense = HalDashboard.getNumber(
                "1.Ramparts:DistanceToDefense", RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        distanceApproachDefense = HalDashboard.getNumber(
                "2.Ramparts:DistanceCrossDefense", 70.0);
        distanceCrossDefense = HalDashboard.getNumber(
                "3.Ramparts:DistanceCrossDefense", RobotInfo.AUTO_DISTANCE_CROSS_DEFENSE);
        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm.start(State.DRIVE_TO_RAMPARTS);
    }

    public void autoPeriodic(double elapsedTime)
	{
        State state = (State)sm.getState();
        dashboard.displayPrintf(
                1, "[%6.3f] %s", elapsedTime, state != null? state.toString(): "DISABLED");
        robot.encoderXPidCtrl.displayPidInfo(2);
        robot.encoderYPidCtrl.displayPidInfo(4);
        robot.gyroTurnPidCtrl.displayPidInfo(6);
        robot.arm.displayDebugInfo(8);

        if (sm.isReady())
        {
            state = (State) sm.getState();

            switch(state)
            {
            case DRIVE_TO_RAMPARTS:
                /*
                 * drive to defense fast, put arms up
                 */
                robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                robot.pidDrive.setTarget(0.0, distanceToDefense, 0.0, false, event, 2.0);
//                robot.arm.setPosition(RobotInfo.ARM_OUT_POSITION);
                robot.arm.setPower(1.0, 1.0);
                sm.addEvent(event);
                sm.waitForEvents(State.APPROACH_RAMPARTS);
                break;

            case APPROACH_RAMPARTS:
                /*
                 * drive forward to the ramparts slowly
                 */
                robot.encoderYPidCtrl.setOutputRange(-.3, .3);
                robot.pidDrive.setTarget(0.0, distanceApproachDefense, 0.0, false, event, 2.0);
                sm.addEvent(event);
                sm.waitForEvents(State.LOWER_ARMS);
                break;

            case LOWER_ARMS:
                /*
                 * lower arms to push up robot
                 */
//                robot.arm.setPosition(RobotInfo.ARM_DOWN_POSITION, event, 1.0);
                robot.arm.setPower(1.0, false);
                timer.set(1.0, event);
                sm.addEvent(event);
                sm.waitForEvents(State.OVER_RAMPARTS);
                break;

            case OVER_RAMPARTS:
                /*
                 * drive over ramparts 30%
                 */
                robot.arm.setPower(1.0, 1.0);
                robot.pidDrive.setTarget(0.0, distanceCrossDefense, 0.0, false, event, 1.0);
                sm.addEvent(event);
                sm.waitForEvents(State.DONE);
                break;

            case DONE:
            default:
                //
                // stop
                //
                robot.arm.setPosition(RobotInfo.ARM_UP_POSITION);
                sm.stop();
                break;
            }
        }
    }   //autoPeriodic
}
