package frc492;

import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;

/*
Autonomous for portcullis
rebecca cheng~~
 */

public class AutoPortcullis implements AutoStrategy
{
	// constants
    public static final double DISTANCE_TO_DEFENSE	= 50;
    public static final double DISTANCE_TO_PORT 	= 20;
    public static final double ARM_TO_NEUTRAL		= -1.0;
    public static final double ELEVATOR_TO_NEUTRAL 	= 1.0;	
    public static final double ARM_PORT_LOW         = 0.0;
    public static final double ARM_PORT_UP          = 0.0;
    public static final double DISTANCE_UNDER_PORT  = 0.0;
    public static final double DISTANCE_PAST_PORT   = 0.0;
	
	// variables
	private static final String moduleName = "AutoPortcullis";
	private Robot robot;
	private TrcEvent driveEvent;
	private TrcEvent armEvent;
	private TrcStateMachine sm;
	
	// state machine
	private enum State
	{
		START,
		STATE_DRIVE_TO_DEFENSE,
		STATE_DRIVE_TO_PORT,
		STATE_LIFT_GATE,
		STATE_DRIVE_UNDER_PORT,
		STATE_DRIVE_PAST,
		STATE_DONE
	}
	
	// constructor takes: robot, distance from neutral line
	public AutoPortcullis (Robot robot)
	{
		this.robot = robot;
		driveEvent = new TrcEvent(moduleName + ".driveEvent");
        armEvent = new TrcEvent(moduleName + ".armEvent");
        sm = new TrcStateMachine(moduleName + ".sm");
        sm.start(State.START);
	}
	
	public void autoPeriodic(double elapsedTime)
	{
        if (sm.isReady())
        {
            State state = (State) sm.getState();
            switch (state)
            {
            case STATE_DRIVE_TO_DEFENSE:
                //
                // drive up to the defense full speed, lower arms
                //
            	robot.pidDrive.setTarget(0.0, DISTANCE_TO_DEFENSE, 0.0, false, driveEvent, 2.0);
                robot.arm.setPosition(ARM_PORT_LOW);
               
                sm.addEvent(driveEvent);
                sm.waitForEvents(State.STATE_DRIVE_TO_PORT);
                break;

            case STATE_DRIVE_TO_PORT:
                //
                // drive to the portcullius at 30% speed
                //
                robot.encoderYPidCtrl.setOutputRange(-.3, .3);
            	robot.pidDrive.setTarget(0.0, DISTANCE_TO_PORT, 0.0, false, driveEvent, 2.0);
            	
                sm.addEvent(driveEvent);
                sm.waitForEvents(State.STATE_LIFT_GATE);
                break;
                
            case STATE_LIFT_GATE:
            	//
            	// lift the gate
            	//
            	robot.arm.setPosition(ARM_PORT_UP, armEvent, 1.0);
            	
                sm.addEvent(armEvent);
            	sm.waitForEvents(State.STATE_DRIVE_UNDER_PORT);
                break;
            
            case STATE_DRIVE_UNDER_PORT:
            	//
            	// drive slowly under the port at 50% power
            	//
                robot.encoderYPidCtrl.setOutputRange(-.5, .5);
                robot.pidDrive.setTarget(0.0, DISTANCE_UNDER_PORT, 0, false, driveEvent, 2.0);
            	//robot.arm.setPosition(position);
                
            	sm.addEvent(driveEvent);
            	sm.waitForEvents(State.STATE_DRIVE_PAST);
                break;
            	
            case STATE_DRIVE_PAST:
                robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                robot.pidDrive.setTarget(0.0, DISTANCE_PAST_PORT, 0.0, false, driveEvent, 2.0);
                robot.arm.setPosition(ARM_TO_NEUTRAL);
                
                sm.addEvent(driveEvent);
                sm.waitForEvents(State.STATE_DONE);
                break;
            

            case STATE_DONE:
            default:
                //
                // stop 
                //
                
                sm.stop();
                break;
            }
        }
    }   //autoPeriodic
}
