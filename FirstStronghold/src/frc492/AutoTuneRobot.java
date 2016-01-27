package frc492;

import frclibj.TrcDashboard;
import frclibj.TrcEvent;
import frclibj.TrcRobot;
import frclibj.TrcStateMachine;

public class AutoTuneRobot implements TrcRobot.AutoStrategy
{
    private static final String moduleName = "AutoTuneRobot";
    private Robot robot;
    private TrcEvent event;
    private TrcStateMachine sm;
    private Autonomous.TuneMode tuneMode;

    public AutoTuneRobot(Robot robot, Autonomous.TuneMode tuneMode)
    {
        this.robot = robot;
        this.tuneMode = tuneMode;
        event = new TrcEvent(moduleName + ".event");
        sm = new TrcStateMachine(moduleName + ".sm");
        sm.start();
    }   //AutoTuneRobot

    //
    // Implements TrcRobot.AutoStrategy.
    //
    public void autoPeriodic()
    {
        boolean ready = sm.isReady();
        TrcDashboard.textPrintf(1, "%s[%d] = %s",
                moduleName,
                sm.getState(),
                ready? "Ready": "NotReady");

        robot.xPidCtrl.displayPidInfo(2);
        robot.yPidCtrl.displayPidInfo(4);
//        robot.sonarPidCtrl.displayPidInfo(4);
        robot.turnPidCtrl.displayPidInfo(6);

        if (ready)
        {
            int state = sm.getState();
            switch (state)
            {
            case TrcStateMachine.STATE_STARTED:
                switch (tuneMode)
                {
                case TUNEMODE_MOVE_X:
                    robot.pidDrive.setTarget(
                          240.0, 0.0, 0.0,
                          false,
                          event,
                          0.0);
                    break;

                case TUNEMODE_MOVE_Y:
                    robot.pidDrive.setTarget(
                            0.0, 240.0, 0.0,
                            false,
                            event,
                            0.0);
                    break;

                case TUNEMODE_TURN:
                    robot.pidDrive.setTarget(
                            0.0, 0.0, 360.0,
                            false,
                            event,
                            0.0);
                    break;

                case TUNEMODE_SONAR:
                    robot.sonarPidDrive.setTarget(
                            0.0, 8.0, 0.0,
                            false,
                            event,
                            0.0);
                    break;
                }
                sm.addEvent(event);
                sm.waitForEvents(state + 1);
                break;

            default:
                //
                // We are done.
                //
                sm.stop();
            }
        }
    }   //autoPeriodic

}   //class AutoTuneRobot
