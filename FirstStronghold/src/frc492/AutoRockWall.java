package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

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
    private double distanceApproachDefense;
    private double distanceCrossDefense;
    private TrcStateMachine sm;
    private TrcEvent event;
    private TrcTimer timer;

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
        distanceToDefense = HalDashboard.getNumber(
                "1.RockWall:DistanceToDefense", RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        distanceApproachDefense = HalDashboard.getNumber(
                "2.RockWall:DistanceApproachDefense", 70.0);
        distanceCrossDefense = HalDashboard.getNumber(
                "DistanceOverRockWall", RobotInfo.AUTO_DISTANCE_CROSS_DEFENSE);
        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
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
                    robot.pidDrive.setTarget(0.0, distanceToDefense, 0.0, false, event, 2.0);
//                    robot.arm.setPosition(RobotInfo.ARM_OUT_POSITION);
                    robot.arm.setPower(1.0, 1.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.APPROACH_ROCKWALL);
                    break;

                case APPROACH_ROCKWALL:
                    /*
                     * drive to defense
                     */
                    robot.encoderYPidCtrl.setOutputRange(-0.4, 0.4);
                    robot.pidDrive.setTarget(
                            0.0, distanceApproachDefense, 0.0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.LOWER_ARMS);
                    break;

                case LOWER_ARMS:
                    /*
                     * lower arms to push up robot
                     */
                    //robot.arm.setPosition(RobotInfo.ARM_DOWN_POSITION, event, 1.0);
                    robot.arm.setPower(1.0, false);
                    timer.set(1.0, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.OVER_ROCKWALL);
                    break;

                case OVER_ROCKWALL:
                    /*
                     * drive over wall slowly
                     */
                    robot.arm.setPower(1.0, 2.0);
                    robot.encoderYPidCtrl.setOutputRange(-0.7, 0.7);
                    robot.pidDrive.setTarget(0.0, distanceCrossDefense, 0.0, false, event, 5.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // stop
                    //
                    //robot.arm.setPosition(RobotInfo.ARM_UP_POSITION);
                    robot.arm.setPower(-1.0, 2.0);
                    sm.stop();
                    break;
            }
        }
    }   //autoPeriodic
}
