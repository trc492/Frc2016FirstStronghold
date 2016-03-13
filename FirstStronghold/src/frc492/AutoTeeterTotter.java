package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcRobot.AutoStrategy;

public class AutoTeeterTotter implements AutoStrategy 
{
    private static final String moduleName = "AutoTeeterTotter";
    private HalDashboard dashboard = HalDashboard.getInstance();

    private Robot robot;
    private double distanceToTeeterTotter;
    private double distanceApproachTeeterTotter;
    private double distanceOverTeeterTotter;
    private TrcStateMachine sm;
    private TrcEvent event;

    public enum State
    {
        DRIVE_TO_TEETER_TOTTER,
        APPROACH_TEETER_TOTTER,
        LOWER_ARMS,
        OVER_TEETER_TOTTER,
        DONE
    }

    public AutoTeeterTotter(Robot robot)
    {
        this.robot = robot;
        distanceToTeeterTotter = HalDashboard.getNumber(
                "DistanceToTeeterTotter", RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        distanceApproachTeeterTotter = HalDashboard.getNumber(
                "DistanceApproachTeeterTotter", distanceToTeeterTotter + 20.0);
        distanceOverTeeterTotter = HalDashboard.getNumber(
                "DistanceOverTeeterTotter", distanceToTeeterTotter + 68.0);
        sm = new TrcStateMachine (moduleName);
        event = new TrcEvent (moduleName);
        sm.start(State.DRIVE_TO_TEETER_TOTTER);
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
            state = (State)sm.getState();

            switch(state)
            {
                case DRIVE_TO_TEETER_TOTTER:
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceToTeeterTotter, 0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.APPROACH_TEETER_TOTTER);
                    break;

                case APPROACH_TEETER_TOTTER:
                    robot.encoderYPidCtrl.setOutputRange(-0.3, 0.3);
                    robot.pidDrive.setTarget(
                            0.0, distanceToTeeterTotter + 20.0, 0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.LOWER_ARMS);
                    break;

                case LOWER_ARMS:
                    robot.arm.setPosition(RobotInfo.ARM_DOWN_POSITION, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.OVER_TEETER_TOTTER);
                    break;

                case OVER_TEETER_TOTTER:
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceOverTeeterTotter, false, event, 5.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    robot.arm.setPosition(RobotInfo.ARM_UP_POSITION);
                    sm.stop();
                    break;
            }
        }
    }

}//End of class AutoTeeterTotter
