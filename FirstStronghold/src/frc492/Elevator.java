package frc492;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import frclib.FrcCANTalon;
import frclib.FrcRobotBase;
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

    /*
     * Constructor
     */
    public Elevator()
    {
        leftMotor = new FrcCANTalon(RobotInfo.CANID_LEFT_ELEVATOR);
        rightMotor = new FrcCANTalon(RobotInfo.CANID_RIGHT_ELEVATOR);
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        leftMotor.setRevLimitSwitchEnabled(true);
        rightMotor.setRevLimitSwitchEnabled(true);
        leftMotor.setFwdLimitSwitchEnabled(true);
        rightMotor.setFwdLimitSwitchEnabled(true);
        leftMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        rightMotor.ConfigRevLimitSwitchNormallyOpen(false);
        leftMotor.ConfigRevLimitSwitchNormallyOpen(false);
        rightMotor.ConfigFwdLimitSwitchNormallyOpen(false);
        leftMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rightMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        leftMotor.reverseSensor(true);
        rightMotor.reverseSensor(false);

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

        pidMotor = new TrcPidMotor(
                moduleName,
                leftMotor, rightMotor,
                RobotInfo.ELEVATOR_SYNC_GAIN,
                pidCtrl);
        pidMotor.setPositionScale(RobotInfo.ELEVATOR_COUNTS_PER_INCH);
    }

    public void displayDebugInfo(int lineNum)
    {
        pidCtrl.displayPidInfo(lineNum);
    }

    public void setElevatorOverride(boolean enabled)
    {
        elevatorOverride = enabled;
    }
    
    public void zeroCalibrate()
    {
        pidMotor.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
    }

    public void setPower(double power)
    {
        pidMotor.setPower(power);
        FrcRobotBase.getRobotTracer().traceInfo(
                moduleName, "Power=%.2f, lEnc=%.0f, rEnc=%.0f",
                power, leftMotor.getPosition(), rightMotor.getPosition());
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
        */
    }

    public void resetPosition()
    {
        leftMotor.resetPosition();
        rightMotor.resetPosition();
    }   //resetPosition
    
    public void setHeight(double height)
    {
        pidMotor.setTarget(height, true);
    }

    public void setHeight(double height, TrcEvent event, double timeout)
    {
        pidMotor.setTarget(height, event, timeout);
    }

    public double getHeight()
    {
        return (leftMotor.getPosition() + rightMotor.getPosition())/2.0/
               RobotInfo.ELEVATOR_COUNTS_PER_INCH;
    }

    public boolean isLeftUpperLimitSwitchActive()
    {
        return leftMotor.isFwdLimitSwitchActive();
    }

    public boolean isLeftLowerLimitSwitchActive()
    {
        return leftMotor.isRevLimitSwitchActive();
    }
    
    public boolean isRightUpperLimitSwitchActive()
    {
        return rightMotor.isFwdLimitSwitchActive();
    }

    public boolean isRightLowerLimitSwitchActive()
    {
        return rightMotor.isRevLimitSwitchActive();
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
