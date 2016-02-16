/*
 * Titan Robotics Framework Library
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.net)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package hallib;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * This class implements a robot drive base that supports 2-motor or 4-motor
 * drive trains. It supports tank drive, arcade drive, mecanum drive and swerve
 * drive. This is a port from the WPILib RobotDrive class and extended with
 * addition features.
 */
public class HalRobotDrive extends RobotDrive
{
    public HalRobotDrive(int leftMotorChannel, int rightMotorChannel)
    {
        super(leftMotorChannel, rightMotorChannel);
    }

    public HalRobotDrive(
            int frontLeftMotor, int rearLeftMotor,
            int frontRightMotor, int rearRightMotor)
    {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    }

    public HalRobotDrive(
            HalMotorController leftMotor, HalMotorController rightMotor)
    {
        super((SpeedController)leftMotor, (SpeedController)rightMotor);
    }

    public HalRobotDrive(
            HalMotorController frontLeftMotor,
            HalMotorController rearLeftMotor,
            HalMotorController frontRightMotor,
            HalMotorController rearRightMotor)
    {
        super((SpeedController)frontLeftMotor,
              (SpeedController)rearLeftMotor,
              (SpeedController)frontRightMotor,
              (SpeedController)rearRightMotor);
    }

    /**
     * This method implements tank drive where leftPower controls the left motors
     * and right power controls the right motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param squaredInput specifies true to square the input values, false otherwise.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void tankDrive(
            double leftPower, double rightPower, boolean squaredInput, boolean inverted)
    {
        if (inverted)
        {
            double swap = leftPower;
            leftPower = -rightPower;
            rightPower = -swap;
        }

        super.tankDrive(leftPower, rightPower, squaredInput);
    }   //tankDrive

    /**
     * This method implements arcade drive where drivePower controls how fast
     * the robot goes in the y-axis and turnPower controls how fast it will
     * turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     * @param squaredInput specifies true to square the input values, false otherwise.
     */
    public void arcadeDrive(
            double drivePower, double turnPower, boolean squaredInput)
    {
        //
        // The arcadeDrive in WPILib has the turnPower reversed. It turns left when turnPower
        // is positive. Our convention is turning right when turnPower is positive. So we need
        // to override WPILib and negate turn power here.
        //
        super.arcadeDrive(drivePower, -turnPower, squaredInput);
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast
     * the robot goes in the y-axis and turnPower controls how fast it will
     * turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     * @param squaredInput specifies true to square the input values, false otherwise.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void arcadeDrive(
            double drivePower, double turnPower, boolean squaredInput, boolean inverted)
    {
        if (inverted)
        {
            drivePower = -drivePower;
        }

        this.arcadeDrive(drivePower, turnPower, squaredInput);
    }   //arcadeDrive

    /**
     * This method implements mecanum drive where x controls how fast the robot will
     * go in the x direction, and y controls how fast the robot will go in the y direction.
     * Rotation controls how fast the robot rotates and gyroAngle specifies the heading
     * the robot should maintain.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation,
                                       boolean inverted, double gyroAngle)
    {
        if (inverted)
        {
            x = -x;
            y = -y;
        }
        
        super.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where x controls how fast the robot will
     * go in the x direction, and y controls how fast the robot will go in the y direction.
     * Rotation controls how fast the robot rotates and gyroAngle specifies the heading
     * the robot should maintain.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted)
    {
        mecanumDrive_Cartesian(x, y, rotation, inverted, 0.0);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot
     * will go in the given direction and how fast it will robote.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation,
                                   boolean inverted)
    {
        if (inverted)
        {
            direction += 180.0;
            direction %= 360.0;
        }

        super.mecanumDrive_Polar(magnitude, direction, rotation);
    }   //mecanumDrive_Polar

}   //HalRobotDrive
