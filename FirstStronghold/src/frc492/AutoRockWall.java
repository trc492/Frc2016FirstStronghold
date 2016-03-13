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
    private double distanceToRockWall;
    private double distanceApproachRockWall;
    private double distanceOverRockWall;
    private TrcStateMachine sm;
    private TrcEvent event;

    // state machine
    private enum State
    {
        DRIVE_TO_ROCKWALL,
        APPROACH_ROCKWALL,
        LOWER_ARMS,
        OVER_ROCKWALL,
        DONE
    }

    // constructor takes: robot, distance from neutral line
    public AutoRockWall (Robot robot)
    {
        this.robot = robot;
        distanceToRockWall = HalDashboard.getNumber(
                "DistanceToRockWall", RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        distanceApproachRockWall = HalDashboard.getNumber(
                "DistanceApproachRockWall", distanceToRockWall + 20.0);
        distanceOverRockWall = HalDashboard.getNumber(
                "DistanceOverRockWall", distanceToRockWall + 48.0);
        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        sm.start(State.DRIVE_TO_ROCKWALL);
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
                case DRIVE_TO_ROCKWALL:
                    /*
                     * drive to defense
                     */
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceToRockWall, 0.0, false, event, 2.0);
                    robot.arm.setPosition(RobotInfo.ARM_OUT_POSITION);
                    sm.addEvent(event);
                    sm.waitForEvents(State.APPROACH_ROCKWALL);
                    break;

                case APPROACH_ROCKWALL:
                    /*
                     * drive to defense
                     */
                    robot.encoderYPidCtrl.setOutputRange(-0.3, 0.3);
                    robot.pidDrive.setTarget(
                            0.0, distanceToRockWall + 20.0, 0.0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.LOWER_ARMS);
                    break;

                case LOWER_ARMS:
                    /*
                     * lower arms to push up robot
                     */
                    robot.arm.setPosition(RobotInfo.ARM_DOWN_POSITION, event, 1.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.OVER_ROCKWALL);
                    break;

                case OVER_ROCKWALL:
                    /*
                     * drive over wall slowly
                     */
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceOverRockWall, 0.0, false, event);
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
