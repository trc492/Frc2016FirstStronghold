package frc492;

//import frclibj.TrcDashboard;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot.AutoStrategy;
import trclib.TrcStateMachine;

/*
Autonomous for ramparts
rebecca cheng~~
 */

public class AutoRamparts implements AutoStrategy
{
	public static final double DISTANCE_TO_DEFENSE				= 50;
    public static final double DISTANCE_TO_RAMPARTS				= 20;
    public static final double SMALL_DISTANCE_OVER_RAMPARTS 	= 0.0;
    public static final double DISTANCE_OVER_RAMPARTS			= 0.0;
    public static final double ARM_TO_NEUTRAL					= 0.0;
    public static final double ELEVATOR_TO_NEUTRAL				= 0.0;
    public static final double DISTANCE_PAST_RAMPARTS			= 0.0;
	
	// variables
	private static final String moduleName = "AutoRamparts";
	//private HalDashboard dashboard = HalDashboard.getInstance();
	private Robot robot;
	private TrcEvent driveEvent;
	private TrcEvent elevatorEvent;
	private TrcEvent armEvent;
	private TrcStateMachine sm;
	
	// state machine
	private enum State
	{
		DRIVE_TO_DEFENSE,
		DRIVE_TO_RAMPARTS,
		LOWER_ARMS,
		DRIVE_FWD,
		RAISE_ARMS,
		DRIVE_OVER_RAMPARTS,
		DONE;
	}
	
	// constructor takes: robot, distance from neutral line
	public AutoRamparts (Robot robot)
	{
		this.robot = robot;
		driveEvent = new TrcEvent(moduleName + ".driveEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        armEvent = new TrcEvent(moduleName + ".armEvent");
        sm = new TrcStateMachine(moduleName + ".sm");
        sm.start(State.DRIVE_TO_DEFENSE);
	}
	
	public void autoPeriodic(double elapsedTime)
	{
        /*TrcDashboard.textPrintf(1, "%s[%d] = %s",
                moduleName,
                sm.getState(),
                ready? "Ready": "NotReady");*/

        if (sm.isReady())
        {
            State state = (State) sm.getState();
            switch(state)
            {
            case DRIVE_TO_DEFENSE:
            	/*
            	 * drive to defense fast, put arms up, elevator down
            	 */            	
            	robot.pidDrive.setTarget(0.0, DISTANCE_TO_DEFENSE, 0.0, false, driveEvent, 2.0);
            	robot.arm.setPosition(RobotInfo.ARM_UP_POSITION);
            	robot.elevator.setHeight(RobotInfo.ELEVATOR_MIN_HEIGHT);
            	
            	sm.addEvent(driveEvent);
            	sm.waitForEvents(State.DRIVE_TO_RAMPARTS, 0.0, true);
                break;
            	
            case DRIVE_TO_RAMPARTS:
            	/*
            	 * drive forward to the ramparts slowly
            	 */
            	robot.encoderYPidCtrl.setOutputRange(-.3, .3);
            	
            	robot.pidDrive.setTarget(0.0, DISTANCE_TO_RAMPARTS, 0.0, false, driveEvent, 2.0);
            	
            	sm.addEvent(driveEvent);
            	sm.waitForEvents(State.LOWER_ARMS, 0.0, true);
                break;
                
            case LOWER_ARMS:
            	/*
            	 * lower arms to push up robot
            	 */            	
            	robot.arm.setPosition(RobotInfo.ARM_DOWN_POSITION, armEvent, 1.0);
            	
            	sm.addEvent(armEvent);
            	sm.waitForEvents(State.DRIVE_FWD, 0.0, true);
                break;
                
            case DRIVE_FWD:
            	/*
            	 * drive over ramparts slowly
            	 */
            	robot.encoderYPidCtrl.setOutputRange(-.3, .3);
            	
            	robot.pidDrive.setTarget(0.0, SMALL_DISTANCE_OVER_RAMPARTS, 0.0, false, driveEvent, 1.0);
            	
            	sm.waitForEvents(State.RAISE_ARMS, 0.0, true);
                break;
                
            case RAISE_ARMS:
            	/*
            	 * raise arms 
            	 */
                robot.arm.setPosition(RobotInfo.ARM_UP_POSITION, armEvent, 1.0);
                sm.waitForEvents(State.DRIVE_OVER_RAMPARTS, 0.0, true);
                break;
                
            case DRIVE_OVER_RAMPARTS:
            	/*
            	 * drive over and past the ramparts
            	 */
            	robot.pidDrive.setTarget(0.0, DISTANCE_OVER_RAMPARTS, 0.0, false, driveEvent, 2.0);
            	sm.waitForEvents(State.DONE, 0.0, true);
                break;
            	
            case DONE:
            default:
                //
                // stop (in the name of love)
                //
            	robot.arm.setPosition(ARM_TO_NEUTRAL);
                robot.elevator.setHeight(ELEVATOR_TO_NEUTRAL);
                robot.pidDrive.setTarget(0.0, DISTANCE_PAST_RAMPARTS, 0.0, false, driveEvent, 2.0);
                
                sm.stop();
            }
        }
    }   //autoPeriodic
}
