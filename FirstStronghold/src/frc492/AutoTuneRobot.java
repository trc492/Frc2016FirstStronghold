package frc492;

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class AutoTuneRobot implements TrcRobot.AutoStrategy
{
    private enum State
    {
        START,
        DONE
    }   //enum State
    
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
        sm.start(State.START);
    }   //AutoTuneRobot

    //
    // Implements TrcRobot.AutoStrategy.
    //
    public void autoPeriodic(double elapsedTime)
    {
        boolean ready = sm.isReady();
        HalDashboard.getInstance().displayPrintf(1, "%s:%s = %s [%.3f]",
                moduleName,
                sm.getState().toString(),
                ready? "Ready": "NotReady",
                elapsedTime);

        robot.encoderYPidCtrl.displayPidInfo(2);
        robot.gyroTurnPidCtrl.displayPidInfo(4);
//      robot.sonarPidCtrl.displayPidInfo(4);

        if (ready)
        {
            State state = (State)sm.getState();
            switch (state)
            {
            case START:
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
                sm.waitForEvents(State.DONE);
                break;

            case DONE:
            default:
                //
                // We are done.
                //
                sm.stop();
            }
        }
    }   //autoPeriodic

}   //class AutoTuneRobot
