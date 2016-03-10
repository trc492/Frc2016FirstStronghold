package frc492;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import frclib.FrcCANTalon;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;

public class Crane implements TrcPidController.PidInput
{		
    private static final String moduleName = "Crane";
    private HalDashboard dashboard = HalDashboard.getInstance();

    private FrcCANTalon winchMotor;
    private TrcPidController winchPidCtrl;
    private TrcPidMotor winchPidMotor;

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
        winchMotor.reverseSensor(false);
        winchPidCtrl = new TrcPidController(
                moduleName,
                RobotInfo.WINCH_KP,
                RobotInfo.WINCH_KI,
                RobotInfo.WINCH_KD,
                RobotInfo.WINCH_KF,
                RobotInfo.WINCH_TOLERANCE,
                RobotInfo.WINCH_SETTLING,
                this);
        winchPidMotor = new TrcPidMotor(moduleName + ".winch", winchMotor, winchPidCtrl);
        winchPidMotor.setPositionScale(RobotInfo.WINCH_INCHES_PER_COUNT);

        //
        // Crane has a motor, an encoder, lower and upper limit switches.
        // It can do full PID control.
        //
        craneMotor = new FrcCANTalon(RobotInfo.CANID_CRANE);
        craneMotor.setInverted(true);
        craneMotor.ConfigRevLimitSwitchNormallyOpen(true);
        craneMotor.ConfigFwdLimitSwitchNormallyOpen(true);
        craneMotor.setLimitSwitchesSwapped(true);
        craneMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        craneMotor.reverseSensor(true);
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
        cranePidMotor = new TrcPidMotor(moduleName + ".crane", craneMotor, cranePidCtrl);
        cranePidMotor.setPositionScale(RobotInfo.CRANE_INCHES_PER_COUNT);

        //
        // Tilter has a motor, an encoder and a lower limit switch.
        // It can do full PID control.
        //
        tilterMotor = new FrcCANTalon(RobotInfo.CANID_TILTER);
        tilterMotor.setInverted(true);
        tilterMotor.ConfigRevLimitSwitchNormallyOpen(false);
        tilterMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        tilterMotor.setLimitSwitchesSwapped(true);
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
        tilterPidCtrl.setOutputRange(-RobotInfo.TILTER_POWER_LIMIT, RobotInfo.TILTER_POWER_LIMIT);
        tilterPidMotor = new TrcPidMotor(
                moduleName + ".tilter", tilterMotor, tilterPidCtrl);
        tilterPidMotor.setPositionScale(RobotInfo.TILTER_DEGREES_PER_COUNT);
    }

    public void displayDebugInfo(int lineNum)
    {
        dashboard.displayPrintf(
                lineNum, "Tilter: Angle=%.2f, SW=%d/%d",
                getTilterAngle(),
                tilterMotor.isLowerLimitSwitchActive()? 1: 0,
                tilterMotor.isUpperLimitSwitchActive()? 1: 0);
        tilterPidCtrl.displayPidInfo(lineNum + 1);
        dashboard.displayPrintf(
                lineNum + 3, "Crane: Length=%.2f, SW=%d/%d",
                getCraneLength(),
                craneMotor.isLowerLimitSwitchActive()? 1: 0,
                craneMotor.isUpperLimitSwitchActive()? 1: 0);
        cranePidCtrl.displayPidInfo(lineNum + 4);
        dashboard.displayPrintf(lineNum + 6, "Winch: Length=%.2f", getWinchLength());
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
        cranePidMotor.setPower(power);
    }

    public void setWinchPower(double power)
    {
        winchMotor.setPower(power);
    }

    public void setTilterPower(double power)
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

    public void setTilterAngle(double angle)
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
        return cranePidMotor.getPosition();
    }

    public double getWinchLength()
    {
        return winchMotor.getPosition()*RobotInfo.WINCH_INCHES_PER_COUNT;
    }

    public double getTilterAngle()
    {
        return tilterPidMotor.getPosition();
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
        else if (pidCtrl == this.winchPidCtrl)
        {
            value = getWinchLength();
        }

        return value;
    }

}   //class Crane
