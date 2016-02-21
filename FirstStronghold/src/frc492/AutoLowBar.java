package frc492;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class AutoLowBar implements TrcRobot.AutoStrategy
{
    private static final String moduleName = "AutoLowBar";
    private Robot robot;
    private TrcStateMachine sm;
    private TrcEvent event;
    
    private enum State
    {
        DRIVE_FAST,
        DRIVE_SLOW,
        DONE
    }

    public AutoLowBar(Robot robot)
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
                  robot.pidDrive.setTarget(0.0, 36.0, 0.0, false, event); // 3 feet
                  sm.addEvent(event);
                  sm.waitForEvents(State.DRIVE_SLOW);
                  break;
                  
              case DRIVE_SLOW:
                  robot.encoderYPidCtrl.setOutputRange(-0.3, 0.3);
                  robot.pidDrive.setTarget(0.0, 108.0, 0.0, false, event); // 9 feet
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

}   //class AutoLowBar
