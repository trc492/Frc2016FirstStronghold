package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;

/*
Autonomous for portcullis
rebecca cheng~~
 */

public class AutoPortcullis implements AutoStrategy
{
    // variables
    private static final String moduleName = "AutoPortcullis";
    private HalDashboard dashboard = HalDashboard.getInstance();

    private Robot robot;
    private double distanceToDefense;
    private double armOutPosition;
    private double distanceApproachDefense;
    private double distanceCrossDefense;
    private TrcStateMachine sm;
    private TrcEvent event;

    // state machine
    private enum State
    {
        DRIVE_TO_PORTCULLIS,
        APPROACH_PORTCULLIS,
        UNDER_PORTCULLIS,
        DONE
    }

    // constructor takes: robot, distance from neutral line
    public AutoPortcullis (Robot robot)
    {
        this.robot = robot;
        distanceToDefense = HalDashboard.getNumber(
                "1.Portcullis:DistanceToDefense", RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        armOutPosition = HalDashboard.getNumber(
                "2.Portcullis:ArmOutPosition", RobotInfo.ARM_OUT_POSITION);
        distanceApproachDefense = HalDashboard.getNumber(
                "3.Portcullis:DistanceApproachDefense", 70.0);
        distanceCrossDefense = HalDashboard.getNumber(
                "4.Portcullis:DistanceCrossDefense", RobotInfo.AUTO_DISTANCE_CROSS_DEFENSE);

        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        sm.start(State.DRIVE_TO_PORTCULLIS);
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
            switch (state)
            {
                case DRIVE_TO_PORTCULLIS:
                    //
                    // drive up to the defense full speed, lower arms
                    //
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceToDefense, 0.0, false, event, 2.0);
//                    robot.arm.setPosition(armOutPosition);
                    robot.arm.setPower(1.0, armOutPosition);
                    sm.addEvent(event);
                    sm.waitForEvents(State.APPROACH_PORTCULLIS);
                    break;

                case APPROACH_PORTCULLIS:
                    //
                    // drive to the portcullis at 30% speed
                    //
                    robot.encoderYPidCtrl.setOutputRange(-.3, .3);
                    robot.pidDrive.setTarget(0.0, distanceApproachDefense, 0.0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.UNDER_PORTCULLIS);
                    break;

                case UNDER_PORTCULLIS:
                    //
                    // lift the gate and drive slowly under the port at 50% power
                    //
                    robot.encoderYPidCtrl.setOutputRange(-.5, .5);
//                    robot.arm.setPosition(RobotInfo.ARM_UP_POSITION);
                    robot.arm.setPower(-1.0, 2.0);
                    robot.pidDrive.setTarget(0.0, distanceCrossDefense, 0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // stop
                    //
                    sm.stop();
                    break;
            }
        }
    }   //autoPeriodic
}
