package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class AutoMoat implements TrcRobot.AutoStrategy
{
    private static final String moduleName = "AutoMoat";
    private HalDashboard dashboard = HalDashboard.getInstance();

    private Robot robot;
    private double distanceCrossDefense;
    private TrcStateMachine sm;
    private TrcEvent event;

    private enum State
    {
        DRIVE_OVER_MOAT,
        DONE
    }

    public AutoMoat(Robot robot)
    {
        this.robot = robot;
        distanceCrossDefense = HalDashboard.getNumber("1.Moat:DistanceCrossDefense", 144.0);
        sm = new TrcStateMachine(moduleName);
        event = new TrcEvent(moduleName);
        sm.start(State.DRIVE_OVER_MOAT);
     }   //Autonomous

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
                case DRIVE_OVER_MOAT:
                    robot.encoderYPidCtrl.setOutputRange(-0.6, 0.6);
                    robot.pidDrive.setTarget(0.0, distanceCrossDefense, 0.0, false, event);
//                    robot.arm.setPosition(RobotInfo.ARM_OUT_POSITION);
                    robot.arm.setPower(1.0,  1.0);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    robot.driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    }   //autoPeriodic

}   //class AutoMoat
