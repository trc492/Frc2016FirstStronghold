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

}   //HalRobotDrive
