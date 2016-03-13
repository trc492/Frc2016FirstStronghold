package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;

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
    private double distanceToRamparts;
    private double distanceApproachRamparts;
    private double distanceOverRamparts;
    private TrcStateMachine sm;
    private TrcEvent event;

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
        distanceToRamparts = HalDashboard.getNumber(
                "DistanceToRamparts", RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        distanceApproachRamparts = HalDashboard.getNumber(
                "DistanceCrossRamparts", distanceToRamparts + 20.0);
        distanceOverRamparts = HalDashboard.getNumber(
                "DistanceOverRamparts", distanceToRamparts + 60.0);
        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
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
                robot.pidDrive.setTarget(0.0, distanceToRamparts, 0.0, false, event, 2.0);
                robot.arm.setPosition(RobotInfo.ARM_OUT_POSITION);
                sm.addEvent(event);
                sm.waitForEvents(State.APPROACH_RAMPARTS);
                break;

            case APPROACH_RAMPARTS:
                /*
                 * drive forward to the ramparts slowly
                 */
                robot.encoderYPidCtrl.setOutputRange(-.3, .3);
                robot.pidDrive.setTarget(0.0, distanceApproachRamparts, 0.0, false, event, 2.0);
                sm.addEvent(event);
                sm.waitForEvents(State.LOWER_ARMS);
                break;

            case LOWER_ARMS:
                /*
                 * lower arms to push up robot
                 */
                robot.arm.setPosition(RobotInfo.ARM_DOWN_POSITION, event, 1.0);
                sm.addEvent(event);
                sm.waitForEvents(State.OVER_RAMPARTS);
                break;

            case OVER_RAMPARTS:
                /*
                 * drive over ramparts 30%
                 */
                robot.pidDrive.setTarget(0.0, distanceOverRamparts, 0.0, false, event, 1.0);
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
