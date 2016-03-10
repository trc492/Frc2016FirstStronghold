package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;

/*
Autonomous for rock wall
rebecca cheng~~
 */

public class AutoRockWall implements AutoStrategy
{
    // variables
    private static final String moduleName = "AutoRockWall";
    private HalDashboard dashboard = HalDashboard.getInstance();

    private Robot robot;
    private double distanceToDefense;
    private TrcStateMachine sm;
    private TrcEvent event;

    // state machine
    private enum State
    {
        DRIVE_TO_DEFENSE,
        DRIVE_TO_WALL,
        LOWER_ARMS,
        DRIVE_OVER_WALL,
        DONE
    }

    // constructor takes: robot, distance from neutral line
    public AutoRockWall (Robot robot)
    {
        this.robot = robot;
        distanceToDefense = HalDashboard.getNumber(
                RobotInfo.AUTOKEY_DISTANCE_TO_DEFENSE, RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        sm.start(State.DRIVE_TO_DEFENSE);
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
                case DRIVE_TO_DEFENSE:
                    /*
                     * drive to defense
                     */
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceToDefense, 0.0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DRIVE_TO_WALL);
                    break;

                case DRIVE_TO_WALL:
                    /*
                     * drive forward to the wall slowly
                     */
                    robot.encoderYPidCtrl.setOutputRange(-.3, .3);
                    robot.pidDrive.setTarget(0.0, distanceToDefense + 20.0, 0.0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.LOWER_ARMS);
                    break;

                case LOWER_ARMS:
                    /*
                     * lower arms to push up robot
                     */
                    robot.arm.setPosition(RobotInfo.ARM_DOWN_POSITION, event, 1.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DRIVE_OVER_WALL);
                    break;

                case DRIVE_OVER_WALL:
                    /*
                     * drive over wall slowly
                     */
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceToDefense + 68.0, 0.0, false, event);
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
