package frc492;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc492.Autonomous.AutoMode;
import frc492.Autonomous.TuneMode;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class Test implements TrcRobot.RobotMode
{
    private HalDashboard dashboard = HalDashboard.getInstance();
    private Robot robot;
    private TrcStateMachine sm;
    private TrcEvent event;
    private TrcTimer timer;
    
    private enum State
    {
        START,
        DONE
    }

    public Test(Robot robot)
    {
        this.robot = robot;
        event = new TrcEvent("Test");
        timer = new TrcTimer("Test");
        sm = new TrcStateMachine("Test");
     }   //Autonomous

    //
    // Implements TrcRobot.RobotMode.
    //
    public void startMode()
    {
        sm.start(State.DONE);
    }   //startMode

    public void stopMode()
    {
    }   //stopMode

    public void runPeriodic(double elapsedTime)
    {
        LiveWindow.run();
    }   //runPeriodic

    public void runContinuous(double elapsedTime)
    {
        if (sm.isReady())
        {
            State state = (State)sm.getState();
            
            switch (state)
            {
                case START:
                    robot.driveBase.mecanumDrive_Cartesian(0.0, 0.3, 0.0);
                    timer.set(2.0, event);
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
