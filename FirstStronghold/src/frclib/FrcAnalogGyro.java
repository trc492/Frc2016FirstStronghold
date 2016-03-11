package frclib;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import hallib.HalGyro;
import hallib.HalUtil;
import trclib.TrcSensor.SensorData;

public class FrcAnalogGyro extends AnalogGyro implements HalGyro
{
    public FrcAnalogGyro(int port)
    {
        super(port);
    }   //FrcAnalogGyro

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
        //
        // X-axis is not supported.
        //
    }   //setXInverted

    /**
     * This method inverts the y-axis. This is useful if the orientation of
     * the gyro y-axis is such that the data goes the wrong direction.
     *
     * @param inverted specifies true to invert y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        //
        // Y-axis is not supported.
        //
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
        //
        // X-axis is not supported.
        //
        return null;
    }   //getXRotationRate

    /**
     * This method returns the rotation rate on the y-axis.
     *
     * @return Y rotation rate.
     */
    public SensorData getYRotationRate()
    {
        //
        // Y-axis is not supported.
        //
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
        //
        // X-axis is not supported.
        //
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
        //
        // Y-axis is not supported.
        //
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
        //
        // X-axis is not supported.
        //
    }   //resetXIntegrator

    /**
     * This method resets the integrator on the y-axis.
     */
    public void resetYIntegrator()
    {
        //
        // Y-axis is not supported.
        //
    }   //resetYIntegrator

    /**
     * This method resets the integrator on the z-axis.
     */
    public void resetZIntegrator()
    {
        reset();
    }   //resetZIntegrator

}   //class FrcAnalogGyro
