package frc492;

import frclib.FrcCANTalon;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcPidController.PidInput;

public class Crane implements TrcPidController.PidInput
{		
	// Crane Motor PID
	private static final double CRANE_KP = 0.0;
	private static final double CRANE_KI = 0.0;
	private static final double CRANE_KD = 0.0;
	private static final double CRANE_KF = 0.0;
	private static final double CRANE_TOLERANCE = 0.0;
	private static final double CRANE_SETTLING = 0.0;
	
	// Crane Tilt Motor PID
	private static final double CRANE_TILT_KP = 0.0;
	private static final double CRANE_TILT_KI = 0.0;
	private static final double CRANE_TILT_KD = 0.0;
	private static final double CRANE_TILT_KF = 0.0;
	private static final double CRANE_TILT_TOLERANCE = 0.0;
	private static final double CRANE_TILE_SETTLING = 0.0;
	
	// Others 
	private static final double CRANE_INCHES_PER_CLICK = 0.0;
	private static final double CRANE_TILT_INCHES_PER_CLICK = 0.0;
	private static final double CRANE_MAX_HEIGHT = 0.0;
	private static final double CRANE_MIN_HEIGHT = 0.0;	
	private static final double CRANE_TILT_MAX_ANGLE = 0.0;
	private static final double CRANE_TILT_MIN_ANGLE = 0.0;
	
	private static final String moduleName = "Crane";
    private FrcCANTalon craneMotor;
    private FrcCANTalon tiltMotor;
    private TrcPidController cranePidCtrl;
    private TrcPidController tiltPidCtrl;
    private TrcPidMotor cranePidMotor;
    private TrcPidMotor tiltPidMotor;
    private boolean craneOverride;
    private boolean tiltOverride;
    private double lastHeight = 0.0;
    private double lastAngle = 0.0;
    
    /*
     * Constructor
     */
    public Crane()
    {
    	craneMotor = new FrcCANTalon(RobotInfo.CANID_CRANE);
    	tiltMotor = new FrcCANTalon(RobotInfo.CANID_CRANE_TILTER);
    	craneMotor.setInverted(true);
        tiltMotor.setInverted(true);
        craneMotor.reverseSensor(true);
        tiltMotor.reverseSensor(true);
        craneMotor.enableBrakeMode(true);
        tiltMotor.enableBrakeMode(true);
        
        cranePidCtrl = new TrcPidController(
                moduleName,
                CRANE_KP,
                CRANE_KI,
                CRANE_KD,
                CRANE_KF,
                CRANE_TOLERANCE,
                CRANE_SETTLING,
                this);
        
        tiltPidCtrl = new TrcPidController(
                moduleName,
                CRANE_TILT_KP,
                CRANE_TILT_KI,
                CRANE_TILT_KD,
                CRANE_TILT_KF,
                CRANE_TILT_TOLERANCE,
                CRANE_TILE_SETTLING,
                this);
        
        cranePidCtrl.setAbsoluteSetPoint(true);
        tiltPidCtrl.setAbsoluteSetPoint(true);
        craneMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        craneMotor.ConfigRevLimitSwitchNormallyOpen(false);
        tiltMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        tiltMotor.ConfigRevLimitSwitchNormallyOpen(false);
        
        cranePidMotor = new TrcPidMotor(
                moduleName,
                craneMotor, tiltMotor,
                cranePidCtrl);
        
        tiltPidMotor = new TrcPidMotor(
                moduleName,
                craneMotor, tiltMotor,
                tiltPidCtrl);
        
        cranePidMotor.setPositionScale(CRANE_INCHES_PER_CLICK);
        tiltPidMotor.setPositionScale(CRANE_TILT_INCHES_PER_CLICK);
    }
    
    public void displayDebugInfo(int lineNum)
    {
        cranePidCtrl.displayPidInfo(lineNum);
        tiltPidCtrl.displayPidInfo(lineNum);
    }
    
    
    /*
     * Overriding
     */
    public void setCraneOverride(boolean enabled)
    {
    	craneOverride = enabled;
    }
    
    public void setTiltOverride(boolean enabled)
    {
    	tiltOverride = enabled;
    }
    
    
    /*
     * Zero Calibration
     */
    public void zeroCraneCalibrate(double calPower)
    {
        cranePidMotor.zeroCalibrate(calPower);
        lastHeight = getHeight();
    }
    
    public void zeroTiltCalibrate(double calPower)
    {
        tiltPidMotor.zeroCalibrate(calPower);
        lastHeight = getHeight();
    }
    
    
    /*
     * Set power
     */
    public void setCranePower(double power)
    {
    	cranePidMotor.setPower(power);
    }
    
    public void setTiltPower(double power)
    {
    	tiltPidMotor.setPower(power);
    }
    
    
    /*
     * Reset position
     */
    public void resetCranePosition()
    {
    	craneMotor.resetPosition();
    }
    
    public void resetTiltPosition()
    {
    	tiltMotor.resetPosition();
    }
    
    
    /*
     * Change height or angle by deltaHeight or deltaAngle
     */
    public void setDeltaHeight(double deltaHeight)
    {
        lastHeight += deltaHeight;
        if (lastHeight > CRANE_MAX_HEIGHT)
        {
            lastHeight = CRANE_MAX_HEIGHT;
        }
        else if (lastHeight < CRANE_MIN_HEIGHT)
        {
            lastHeight = CRANE_MIN_HEIGHT;
        }
        cranePidMotor.setTarget(lastHeight, true);
    }
    
    public void setDeltaAngle(double deltaAngle)
    {
        lastAngle += deltaAngle;
        if (lastAngle > CRANE_TILT_MAX_ANGLE)
        {
        	lastAngle = CRANE_TILT_MAX_ANGLE;
        }
        else if (lastAngle < CRANE_TILT_MIN_ANGLE)
        {
        	lastAngle = CRANE_TILT_MIN_ANGLE;
        }
        tiltPidMotor.setTarget(lastAngle, true);
    }
    
    
    /*
     * Set height or angle
     */
    public void setHeight(double height)
    {
        cranePidMotor.setTarget(height, true);
        lastHeight = height;
    }

    public void setHeight(double height, TrcEvent event, double timeout)
    {
        cranePidMotor.setTarget(height, event, timeout);
    }
    
    public void setAngle(double angle)
    {
        tiltPidMotor.setTarget(angle, true);
        lastAngle = angle;
    }

    public void setAngle(double angle, TrcEvent event, double timeout)
    {
        tiltPidMotor.setTarget(angle, event, timeout);
    }
    
    
    /*
     * Accessors: height, angle, limit switches
     */
    public double getHeight()
    {
        return (craneMotor.getPosition() * CRANE_INCHES_PER_CLICK);
    }
    
    public double getAngle()
    {
        return (tiltMotor.getPosition() * CRANE_INCHES_PER_CLICK);
    }
    
    public boolean isCraneLowerLimitSwitchActive()
    {
        return !craneMotor.isRevLimitSwitchClosed();
    }

    public boolean isTiltLowerLimitSwitchActive()
    {
        return !tiltMotor.isRevLimitSwitchClosed();
    }
    
    public boolean isTiltUpperLimitSwitchActive()
    {
    	return !tiltMotor.isFwdLimitSwitchClosed();
    }
    
    /*
     * Implements TrcPidController.PidInput
     */
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == this.cranePidCtrl)
        {
            value = getHeight();
        }
        
        if (pidCtrl == this.tiltPidCtrl)
        {
            value = getAngle();
        }

        return value;
    }
}

























