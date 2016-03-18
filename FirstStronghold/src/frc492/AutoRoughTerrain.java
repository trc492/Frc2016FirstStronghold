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
    private double distanceCrossDefense;
    private TrcStateMachine sm;
    private TrcEvent event;

    private enum State
    {
        DRIVE_CROSS_ROUGH_TERRAIN,
        DONE
    }

    public AutoRoughTerrain(Robot robot)
    {
        this.robot = robot;
        distanceCrossDefense = HalDashboard.getNumber("1.RoughTerrain:DistanceCrossDefense", 144.0);
        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        sm.start(State.DRIVE_CROSS_ROUGH_TERRAIN);
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
                case DRIVE_CROSS_ROUGH_TERRAIN:
                    robot.encoderYPidCtrl.setOutputRange(-0.7, 0.7);
                    robot.pidDrive.setTarget(0.0, distanceCrossDefense, 0.0, false, event);
                    robot.arm.setPosition(RobotInfo.ARM_OUT_POSITION);// 12 feet
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
