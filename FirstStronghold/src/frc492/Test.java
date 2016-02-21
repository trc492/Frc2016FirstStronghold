package frc492;

import frclib.FrcRobotBase;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class Test implements TrcRobot.RobotMode
{
    private static final String moduleName = "Test";
    private TrcDbgTrace dbgTrace;
    private Robot robot;
    private TrcStateMachine sm;
    private TrcEvent event;
    
    private enum State
    {
        DRIVE_FAST,
        DONE
    }

    public Test(Robot robot)
    {
        dbgTrace = FrcRobotBase.getRobotTracer();
        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine(moduleName);
     }   //Autonomous

    //
    // Implements TrcRobot.RobotMode.
    //
    
    @Override
    public void startMode()
    {
        sm.start(State.DRIVE_FAST);
    }   //startMode

    @Override
    public void stopMode()
    {
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
//        LiveWindow.run();
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        dbgTrace.traceInfo(moduleName, "State: %s", sm.getState().toString());
        robot.encoderYPidCtrl.printPidInfo();
        if (sm.isReady())
        {
            State state = (State)sm.getState();
            
            switch (state)
            {
                case DRIVE_FAST:
                    robot.encoderYPidCtrl.setOutputRange(-0.6, 0.6);
                    robot.pidDrive.setTarget(0.0, 144.0, 0.0, false, event); // 12 feet
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
    }   //runContinuous

}   //class Test
