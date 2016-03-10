package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class AutoRoughTerrain implements TrcRobot.AutoStrategy
{
    private static final String moduleName = "AutoRoughTerrain";
    private HalDashboard dashboard = HalDashboard.getInstance();

    private Robot robot;
    private TrcStateMachine sm;
    private TrcEvent event;

    private enum State
    {
        DRIVE_OVER_DEFENSE,
        DONE
    }

    public AutoRoughTerrain(Robot robot)
    {
        this.robot = robot;
        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        sm.start(State.DRIVE_OVER_DEFENSE);
     }   //AutoRoughTerrain

    //
    // Implements TrcRobot.AutoStrategy.
    //

    @Override
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

            switch (state)
            {
                case DRIVE_OVER_DEFENSE:
                    robot.encoderYPidCtrl.setOutputRange(-0.7, 0.7);
                    robot.pidDrive.setTarget(0.0, 144.0, 0.0, false, event); // 12 feet
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;
                    
                case DONE:
                default:
                    sm.stop();
                    break;
            }
        }
    }   //autoPeriodic

}   //class AutoRoughTerrain
