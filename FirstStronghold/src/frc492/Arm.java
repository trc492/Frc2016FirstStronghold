package frc492;

import frclib.FrcCANTalon;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPidMotor;
import trclib.TrcUtil;

public class Arm implements TrcPidController.PidInput
{
    private static final String moduleName = "Arm";
    private FrcCANTalon leftMotor;
    private FrcCANTalon rightMotor;
    private TrcPidController pidCtrl;
    private TrcPidMotor pidMotor;

    public Arm()
    {
        leftMotor = new FrcCANTalon(RobotInfo.CANID_LEFT_ARM);
        rightMotor = new FrcCANTalon(RobotInfo.CANID_RIGHT_ARM);
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
        leftMotor.reverseSensor(true);
        rightMotor.reverseSensor(false);
        leftMotor.enableBrakeMode(true);
        rightMotor.enableBrakeMode(true);
        pidCtrl = new TrcPidController(
                moduleName,
                RobotInfo.ARM_KP,
                RobotInfo.ARM_KI,
                RobotInfo.ARM_KD,
                RobotInfo.ARM_KF,
                RobotInfo.ARM_TOLERANCE,
                RobotInfo.ARM_SETTLING,
                this);
        pidCtrl.setAbsoluteSetPoint(true);
        pidMotor = new TrcPidMotor(
                moduleName,
                leftMotor, rightMotor,
                RobotInfo.ARM_SYNC_GAIN,
                pidCtrl); 
        pidMotor.setPositionScale(RobotInfo.ELEVATOR_INCHES_PER_CLICK);
    }

    public void displayDebugInfo(int lineNum)
    {
        pidCtrl.displayPidInfo(lineNum);
    }

    public void setDeltaPosition(double deltaPosition)
    {
        double targetPosition = TrcUtil.limit(
                getPosition() + deltaPosition,
                RobotInfo.ARM_MIN_POSITION,
                RobotInfo.ARM_MAX_POSITION);
        pidMotor.setTarget(targetPosition, true);
    }

    public void setPosition(double position)
    {
        pidMotor.setTarget(position, true);
    }

    public void setPosition(double position, TrcEvent event, double timeout)
    {
        pidMotor.setTarget(position, event, timeout);
    }

    public double getPosition()
    {
        return (leftMotor.getPosition() + rightMotor.getPosition())*RobotInfo.ARM_SCALE/2.0;
    }

    public void setPower(double power)
    {
        pidMotor.setPidPower(
                power,
                RobotInfo.ARM_MIN_POSITION,
                RobotInfo.ARM_MAX_POSITION,
                true);
    }
    //
    // Implements TrcPidController.PidInput.
    //

    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == this.pidCtrl)
        {
            value = getPosition();
        }

        return value;
    }   //getInput

}   //class Arm
