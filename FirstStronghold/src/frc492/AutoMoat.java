package frc492;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class AutoMoat implements TrcRobot.AutoStrategy
{
    private static final String moduleName = "AutoRoughTerrain";
    private Robot robot;
    private TrcStateMachine sm;
    private TrcEvent event;
    
    private enum State
    {
        DRIVE_FAST,
        DONE
    }

    public AutoMoat(Robot robot)
    {
        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine(moduleName);
        sm.start(State.DRIVE_FAST);
     }   //Autonomous

    //
    // Implements TrcRobot.AutoStrategy.
    //
    
    @Override
    public void autoPeriodic(double elapsedTime)
    {
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
    }   //autoPeriodic

}   //class AutoMoat
