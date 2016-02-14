/*
 * Lift and lower arms
 */
package frc492;

import frclib.FrcCANTalon;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;

public class Elevator implements TrcPidController.PidInput
{
    private static final String moduleName = "Elevator";
    private FrcCANTalon leftMotor;
    private FrcCANTalon rightMotor;
    private TrcPidController pidCtrl;
    private TrcPidMotor pidMotor;
    private boolean elevatorOverride = false;
    private double lastHeight = 0.0;
    private boolean stopped = true;

    /*
     * Constructor
     */
    public Elevator()
    {
        leftMotor = new FrcCANTalon(RobotInfo.CANID_LEFT_ELEVATOR);
        rightMotor = new FrcCANTalon(RobotInfo.CANID_RIGHT_ELEVATOR);
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        leftMotor.reverseSensor(true);
        rightMotor.reverseSensor(false);
        leftMotor.enableBrakeMode(true);
        rightMotor.enableBrakeMode(true);
        pidCtrl = new TrcPidController(
                moduleName,
                RobotInfo.ELEVATOR_KP,
                RobotInfo.ELEVATOR_KI,
                RobotInfo.ELEVATOR_KD,
                RobotInfo.ELEVATOR_KF,
                RobotInfo.ELEVATOR_TOLERANCE,
                RobotInfo.ELEVATOR_SETTLING,
                this);
        pidCtrl.setAbsoluteSetPoint(true);
        leftMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        leftMotor.ConfigRevLimitSwitchNormallyOpen(false);
        rightMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        rightMotor.ConfigRevLimitSwitchNormallyOpen(false);
        pidMotor = new TrcPidMotor(
                moduleName,
                leftMotor, rightMotor,
                RobotInfo.ELEVATOR_SYNC_GAIN,
                pidCtrl);
        pidMotor.setPositionScale(RobotInfo.ELEVATOR_INCHES_PER_CLICK);
        lastHeight = getHeight();
    }

    public void displayDebugInfo(int lineNum)
    {
        pidCtrl.displayPidInfo(lineNum);
    }

    public void setElevatorOverride(boolean enabled)
    {
        elevatorOverride = enabled;
    }
    
    public void zeroCalibrate(double calPower)
    {
        pidMotor.zeroCalibrate(calPower);
        lastHeight = getHeight();
    }

    public void setPower(double power)
    {
        /*
        leftMotor.set(power);
        rightMotor.set(power);
        if (power != 0.0)
        {
            System.out.printf("p=%.2f,lEnc=%.0f,rEnc=%.0f\n",
                power, leftMotor.getPosition(), rightMotor.getPosition());
        }
        */
        pidMotor.setPower(power);
        /*
        if (elevatorOverride)
        {
            pidMotor.setPower(power);
        }
        else
        {
            pidMotor.setPidPower(
                    power,
                    RobotInfo.ELEVATOR_MIN_HEIGHT,
                    RobotInfo.ELEVATOR_MAX_HEIGHT,
                    true);
        }

        if (power == 0.0)
        {
            if (!stopped)
            {
                lastHeight = getHeight();
            }
            stopped = true;
        }
        else
        {
            stopped = false;
        }
        */
    }

    public void resetPosition()
    {
        leftMotor.resetPosition();
        rightMotor.resetPosition();
    }   //resetPosition
    
    public void setDeltaHeight(double deltaHeight)
    {
        lastHeight += deltaHeight;
        if (lastHeight > RobotInfo.ELEVATOR_MAX_HEIGHT)
        {
            lastHeight = RobotInfo.ELEVATOR_MAX_HEIGHT;
        }
        else if (lastHeight < RobotInfo.ELEVATOR_MIN_HEIGHT)
        {
            lastHeight = RobotInfo.ELEVATOR_MIN_HEIGHT;
        }
        pidMotor.setTarget(lastHeight, true);
    }

    public void setHeight(double height)
    {
        pidMotor.setTarget(height, true);
        lastHeight = height;
    }

    public void setHeight(double height, TrcEvent event, double timeout)
    {
        pidMotor.setTarget(height, event, timeout);
    }

    public double getHeight()
    {
        return (leftMotor.getPosition() + rightMotor.getPosition())*
               RobotInfo.ELEVATOR_INCHES_PER_CLICK/2.0;
    }

    public boolean isUpperLimitSwitchActive()
    {
        return !leftMotor.isFwdLimitSwitchClosed() &&
               !rightMotor.isFwdLimitSwitchClosed();
    }

    public boolean isLowerLimitSwitchActive()
    {
        return !leftMotor.isRevLimitSwitchClosed() &&
               !rightMotor.isRevLimitSwitchClosed();
    }

    //
    // Implements TrcPidController.PidInput.
    //

    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == this.pidCtrl)
        {
            value = getHeight();
        }

        return value;
    }   //getInput

}   //class Elevator
