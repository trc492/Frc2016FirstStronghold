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
    private double distanceToDefense;
    private double distanceOverDefense;
    private TrcStateMachine sm;
    private TrcEvent event;

    public enum State
    {
        DRIVE_TO_DEFENSE,
        DRIVE_TO_TEETER_TOTTER,
        LOWER_ARMS,
        DRIVE_OVER_DEFENSE,
        DONE
    }

    public AutoTeeterTotter(Robot robot)
    {
        this.robot = robot;
        distanceToDefense = HalDashboard.getNumber(
                RobotInfo.AUTOKEY_DISTANCE_TO_DEFENSE, RobotInfo.AUTO_DISTANCE_TO_DEFENSE);
        distanceOverDefense = HalDashboard.getNumber(
                RobotInfo.AUTOKEY_DISTANCE_OVER_DEFENSE, RobotInfo.AUTO_DISTANCE_OVER_DEFENSE);
        sm = new TrcStateMachine (moduleName);
        event = new TrcEvent (moduleName);
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
            state = (State)sm.getState();

            switch(state)
            {
                case DRIVE_TO_DEFENSE:
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, distanceToDefense, 0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DRIVE_TO_TEETER_TOTTER);
                    break;

                case DRIVE_TO_TEETER_TOTTER:
                    robot.encoderYPidCtrl.setOutputRange(-0.3, 0.3);
                    robot.pidDrive.setTarget(0.0, distanceToDefense + 20.0, 0, false, event, 2.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.LOWER_ARMS);
                    break;

                case LOWER_ARMS:
                    robot.arm.setPosition(RobotInfo.ARM_GROUND_POSITION, event, 0.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DRIVE_OVER_DEFENSE);
                    break;

                case DRIVE_OVER_DEFENSE:
                    robot.encoderYPidCtrl.setOutputRange(-0.3, 0.3);
                    robot.pidDrive.setTarget(0.0, distanceOverDefense, false, event, 5.0);
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
