package frc492;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import frclib.FrcCANTalon;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;

public class Crane implements TrcPidController.PidInput
{		
	private static final String moduleName = "Crane";


    private FrcCANTalon winchMotor;
    private FrcCANTalon craneMotor;
    private TrcPidController cranePidCtrl;
    private TrcPidMotor cranePidMotor;

    private FrcCANTalon tilterMotor;
    private TrcPidController tilterPidCtrl;
    private TrcPidMotor tilterPidMotor;
    
    /**
     * Constructor: Create an instance of the object.
     */
    public Crane()
    {
        //
        // Winch has a motor and an encoder but no limit switches.
        // The encoder is used to synchronize with the crane motor.
        //
        winchMotor = new FrcCANTalon(RobotInfo.CANID_WINCH);
        winchMotor.setInverted(true);
        winchMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        winchMotor.reverseSensor(true);

        //
        // Crane has a motor, an encoder, lower and upper limit switches.
        // It can do full PID control.
        //
        craneMotor = new FrcCANTalon(RobotInfo.CANID_CRANE);
        craneMotor.setInverted(false);
        craneMotor.ConfigRevLimitSwitchNormallyOpen(true);
        craneMotor.ConfigFwdLimitSwitchNormallyOpen(true);
        craneMotor.setLimitSwitchesSwapped(false);
        craneMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        craneMotor.reverseSensor(false);
        cranePidCtrl = new TrcPidController(
                moduleName,
                RobotInfo.CRANE_KP,
                RobotInfo.CRANE_KI,
                RobotInfo.CRANE_KD,
                RobotInfo.CRANE_KF,
                RobotInfo.CRANE_TOLERANCE,
                RobotInfo.CRANE_SETTLING,
                this);
        cranePidCtrl.setAbsoluteSetPoint(true);
        cranePidMotor = new TrcPidMotor(
                moduleName + ".crane",
                craneMotor, winchMotor,
                RobotInfo.CRANE_SYNC_GAIN, cranePidCtrl);
        //??? What to do about the encoder scale difference between crane and winch motors???
        cranePidMotor.setPositionScale(RobotInfo.CRANE_COUNTS_PER_INCH);

        //
        // Tilter has a motor, an encoder and a lower limit switch.
        // It can do full PID control.
        //
        tilterMotor = new FrcCANTalon(RobotInfo.CANID_TILTER);
        tilterMotor.setInverted(false);
        tilterMotor.ConfigRevLimitSwitchNormallyOpen(false);
        tilterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        tilterMotor.reverseSensor(false);
        tilterPidCtrl = new TrcPidController(
                moduleName,
                RobotInfo.TILTER_KP,
                RobotInfo.TILTER_KI,
                RobotInfo.TILTER_KD,
                RobotInfo.TILTER_KF,
                RobotInfo.TILTER_TOLERANCE,
                RobotInfo.TILTER_SETTLING,
                this);
        tilterPidCtrl.setAbsoluteSetPoint(true);
        tilterPidMotor = new TrcPidMotor(
                moduleName + ".tilter", tilterMotor, tilterPidCtrl);
        tilterPidMotor.setPositionScale(RobotInfo.TILTER_COUNTS_PER_DEGREE);
    }
    
    public void displayDebugInfo(int lineNum)
    {
        cranePidCtrl.displayPidInfo(lineNum);
        tilterPidCtrl.displayPidInfo(lineNum + 2);
    }

    /*
     * Zero Calibration
     */
    public void zeroCalibarateCrane()
    {
        cranePidMotor.zeroCalibrate(RobotInfo.CRANE_CAL_POWER);
    }
    
    public void zeroCalibrateTilter()
    {
        tilterPidMotor.zeroCalibrate(RobotInfo.TILTER_CAL_POWER);
    }
    
    /*
     * Set power
     */
    public void setCranePower(double power)
    {
        craneMotor.setPower(power);
        winchMotor.setPower(power);
//    	cranePidMotor.setPower(power);
    }
    
    public void setTiltPower(double power)
    {
    	tilterPidMotor.setPower(power);
    }
    
    /*
     * Set height or angle
     */
    public void setCraneLength(double length)
    {
        cranePidMotor.setTarget(length, true);
    }

    public void setCraneLength(double length, TrcEvent event, double timeout)
    {
        cranePidMotor.setTarget(length, event, timeout);
    }
    
    public void setTitlerAngle(double angle)
    {
        tilterPidMotor.setTarget(angle, true);
    }

    public void setTilterAngle(double angle, TrcEvent event, double timeout)
    {
        tilterPidMotor.setTarget(angle, event, timeout);
    }
    
    /*
     * Accessors: Crane length, Tilter angle, limit switches
     */
    public double getCraneLength()
    {
        return (craneMotor.getPosition()/RobotInfo.CRANE_COUNTS_PER_INCH);
    }
    
    public double getWinchLength()
    {
        return (winchMotor.getPosition()/RobotInfo.WINCH_COUNTS_PER_INCH);
    }
    
    public double getTilterAngle()
    {
        return (tilterMotor.getPosition()/RobotInfo.TILTER_COUNTS_PER_DEGREE);
    }
    
    public boolean isCraneLowerLimitSwitchActive()
    {
        return craneMotor.isLowerLimitSwitchActive();
    }

    public boolean isCraneUpperLimitSwitchActive()
    {
        return craneMotor.isUpperLimitSwitchActive();
    }

    public boolean isTilterLowerLimitSwitchActive()
    {
        return tilterMotor.isLowerLimitSwitchActive();
    }
    
    /*
     * Implements TrcPidController.PidInput
     */
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == this.cranePidCtrl)
        {
            value = getCraneLength();
        }
        else if (pidCtrl == this.tilterPidCtrl)
        {
            value = getTilterAngle();
        }

        return value;
    }
}   //class Crane
