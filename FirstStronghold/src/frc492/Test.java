package frc492;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc492.Autonomous.AutoMode;
import frc492.Autonomous.TuneMode;
import frclib.FrcRobotBase;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class Test implements TrcRobot.RobotMode
{
    private HalDashboard dashboard = HalDashboard.getInstance();
    private TrcDbgTrace dbgTrace;
    private Robot robot;
    private TrcStateMachine sm;
    private TrcEvent event;
    private TrcTimer timer;
    
    private enum State
    {
        DRIVE_FAST,
        //DRIVE_SLOW,
        DONE
    }

    public Test(Robot robot)
    {
        dbgTrace = FrcRobotBase.getRobotTracer();
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
        sm.start(State.DRIVE_FAST);
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
        /*
        double lfEnc = robot.leftFrontMotor.getPosition();
        double rfEnc = robot.rightFrontMotor.getPosition();
        double lrEnc = robot.leftRearMotor.getPosition();
        double rrEnc = robot.rightRearMotor.getPosition();
        dbgTrace.traceInfo("Test", "Enc: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f, avg=%.0f",
                lfEnc, rfEnc, lrEnc, rrEnc, (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
        */
        dbgTrace.traceInfo("Test", "State: %s", sm.getState().toString());
        robot.encoderYPidCtrl.printPidInfo();
        if (sm.isReady())
        {
            State state = (State)sm.getState();
            
            switch (state)
            {
                case DRIVE_FAST:
                    robot.encoderYPidCtrl.setOutputRange(-0.7, 0.7);
                    robot.pidDrive.setTarget(0.0, 144.0, 0.0, false, event); // 7 feet
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;
                    
                /*case DRIVE_SLOW:
                    robot.encoderYPidCtrl.setOutputRange(-0.5, 0.5);
                    robot.pidDrive.setTarget(0.0, 96.0, 0.0, false, event); // 8 feet
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;
                    */
                    
                case DONE:
                default:
                    robot.driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    }   //runContinuous

}   //class Test
