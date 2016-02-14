package frclib;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import hallib.HalGyro;
import hallib.HalUtil;
import trclib.TrcSensor.SensorData;

public class FrcADXRS450Gyro extends ADXRS450_Gyro implements HalGyro
{
    private double zSign = 1.0;

    
    //
    // Implements HalGyro interface.
    //
    
    /**
     * This method inverts the x-axis. This is useful if the orientation of
     * the gyro x-axis is such that the data goes the wrong direction.
     *
     * @param inverted specifies true to invert x-axis, false otherwise.
     */
    public void setXInverted(boolean inverted)
    {
    }   //setXInverted

    /**
     * This method inverts the y-axis. This is useful if the orientation of
     * the gyro y-axis is such that the data goes the wrong direction.
     *
     * @param inverted specifies true to invert y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
    }   //setYInverted

    /**
     * This method inverts the z-axis. This is useful if the orientation of
     * the gyro z-axis is such that the data goes the wrong direction.
     *
     * @param inverted specifies true to invert z-axis, false otherwise.
     */
    public void setZInverted(boolean inverted)
    {
        zSign = inverted? -1.0: 1.0;
    }   //setZInverted

    /**
     * This method returns the rotation rate on the x-axis.
     *
     * @return X rotation rate.
     */
    public SensorData getXRotationRate()
    {
        return null;
    }   //getXRotationRate

    /**
     * This method returns the rotation rate on the y-axis.
     *
     * @return Y rotation rate.
     */
    public SensorData getYRotationRate()
    {
        return null;
    }   //getYRotationRate

    /**
     * This method returns the rotation rate on the z-axis.
     *
     * @return Z rotation rate.
     */
    public SensorData getZRotationRate()
    {
        return new SensorData(HalUtil.getCurrentTime(), zSign*getRate());
    }   //getZRotationRate

    /**
     * This method returns the heading of the x-axis. If there is an integrator,
     * we call the integrator to get the heading. Else if we have an unwrapper,
     * we call the unwrapper to get the heading else we call the platform dependent
     * gyro to get the raw heading value.
     *
     * @return X heading.
     */
    public SensorData getXHeading()
    {
        return null;
    }   //getXHeading

    /**
     * This method returns the heading of the y-axis. If there is an integrator,
     * we call the integrator to get the heading. Else if we have an unwrapper,
     * we call the unwrapper to get the heading else we call the platform dependent
     * gyro to get the raw heading value.
     *
     * @return Y heading.
     */
    public SensorData getYHeading()
    {
        return null;
    }   //getYHeading

    /**
     * This method returns the heading of the z-axis. If there is an integrator,
     * we call the integrator to get the heading. Else if we have an unwrapper,
     * we call the unwrapper to get the heading else we call the platform dependent
     * gyro to get the raw heading value.
     *
     * @return Z heading.
     */
    public SensorData getZHeading()
    {
        return new SensorData(HalUtil.getCurrentTime(), zSign*getAngle());
    }   //getZHeading

    /**
     * This method resets the integrator on the x-axis.
     */
    public void resetXIntegrator()
    {
    }   //resetXIntegrator

    /**
     * This method resets the integrator on the y-axis.
     */
    public void resetYIntegrator()
    {
    }   //resetYIntegrator

    /**
     * This method resets the integrator on the z-axis.
     */
    public void resetZIntegrator()
    {
        reset();
    }   //resetZIntegrator

}   //class FrcADXRS450Gyro
