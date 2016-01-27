/**
 *  Copyright (c) Titan Robotics Club. All rights reserved.
 *  This module contains the definitions of the TrcRobot class.
 *  Environment: Java for National Instrument RoboRIO based Robot.
 */

package frclibj;

import java.io.InputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.communication.*;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frclibj.TrcDbgTrace;

/**
 *  This class defines and implements the TrcRobot object. The TrcRobot
 *  object implements a cooperative multitasking robot. Different subsystems
 *  register themselves as CoopTasks. TrcRobot uses the TaskMgr to task
 *  switch between different subsystem tasks at various points in the robot
 *  loop. This basically simulates a cooperative multitasking scheduler that
 *  task switches between them in different modes.
 */
public class TrcRobot extends RobotBase
{
    private static final String moduleName = "TrcRobot";
    private static final boolean debugEnabled = false;
    private static final boolean dashboardEnabled = true;
    private TrcDbgTrace dbgTrace = null;

    public static enum RunMode
    {
        INVALID_MODE,
        DISABLED_MODE,
        TEST_MODE,
        AUTO_MODE,
        TELEOP_MODE
    }   //enum RunMode

    public interface RobotMode
    {
        public void start();
        public void stop();
        public void periodic();
        public void continuous();
    }   //interface RobotMode

    public interface AutoStrategy
    {
        public void autoPeriodic();
    }   //interface AutoStrategy

    private String progName;
    private RobotMode teleOpMode;
    private RobotMode autoMode;
    private RobotMode testMode;
    private RobotMode disabledMode;

    /**
     * Constructor.
     */
    public TrcRobot(final String progName)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.progName = progName;
        TrcDashboard.clearTextLines();
    }   //TrcRobot

    public void setupRobotModes(
            RobotMode teleOpMode,
            RobotMode autoMode,
            RobotMode testMode,
            RobotMode disabledMode)
    {
        this.teleOpMode = teleOpMode;
        this.autoMode = autoMode;
        this.testMode = testMode;
        this.disabledMode = disabledMode;
    }   //setupRobotModes

    /**
     * Start a competition.
     * This specific StartCompetition() implements "main loop" behavior like
     * that of the FRC control system in 2008 and earlier, with a primary
     * (slow) loop that is called periodically, and a "fast loop" (a.k.a.
     * "spin loop") that is called as fast as possible with no delay between
     * calls. This code needs to track the order of the field starting to
     * ensure that everything happens in the right order. Repeatedly run the
     * correct method, either Autonomous or TeleOp when the robot is
     * enabled. After running the correct method, wait for some state to
     * change, either the other mode starts or the robot is disabled. Then go
     * back and wait for the robot to be enabled again.
     */
    public void startCompetition()
    {
        final String funcName = "startCompetition";

        System.out.printf(
                TrcDbgTrace.ESC_PREFIX + TrcDbgTrace.SGR_FG_BLACK +
                TrcDbgTrace.ESC_SEP + TrcDbgTrace.SGR_BG_WHITE +
                TrcDbgTrace.ESC_SUFFIX +
                "\n****************************************\n" +
                "Host Name: %s\n" +
                "  Program: %s\n"+
//                " Compiled: %s, %s" +
                "\n****************************************\n" +
                TrcDbgTrace.ESC_NORMAL,
                getHostName(), progName);

        UsageReporting.report(
                tResourceType.kResourceType_Framework,
                tInstances.kFramework_Iterative);
        robotInit();
        //
        // We call this now (not in prestart like default) so that the robot
        // won't enable until the initialization has finished. This is useful
        // because otherwise it's sometimes possible to enable the robot
        // before the code is ready.
        //
        FRCNetworkCommunicationsLibrary.
                FRCNetworkCommunicationObserveUserProgramStarting();
        LiveWindow.setEnabled(false);
        //
        // loop forever, calling the appropriate mode-dependent function
        //
        final double timesliceThreshold = 0.05;
        RunMode prevMode = RunMode.INVALID_MODE;
        RunMode currMode = RunMode.INVALID_MODE;
        double modeStartTime = 0.0;

        while (true)
        {
            prevMode = currMode;
            //
            // Determine the current run mode.
            //
            if (isDisabled())
            {
                currMode = RunMode.DISABLED_MODE;
            }
            else if (isTest())
            {
                currMode = RunMode.TEST_MODE;
            }
            else if (isAutonomous())
            {
                currMode = RunMode.AUTO_MODE;
            }
            else if (isOperatorControl())
            {
                currMode = RunMode.TELEOP_MODE;
            }
            else
            {
                currMode = RunMode.INVALID_MODE;
            }

            if (currMode != prevMode)
            {
                //
                // Detected mode transition.
                //
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(
                            funcName,
                            "Mode Transition: %s->%s.",
                            prevMode.toString(), currMode.toString());
                }
                //
                // Execute all stop tasks for previous mode.
                //
                if (prevMode != RunMode.INVALID_MODE)
                {
                    TrcTaskMgr.executeTaskType(
                            TrcTaskMgr.TaskType.STOP_TASK, prevMode);
                }
                //
                // Stop previous mode.
                // 
                if (prevMode == RunMode.DISABLED_MODE && disabledMode != null)
                {
                    disabledMode.stop();
                }
                else if (prevMode == RunMode.TEST_MODE && testMode != null)
                {
                    testMode.stop();
                }
                else if (prevMode == RunMode.AUTO_MODE && autoMode != null)
                {
                    autoMode.stop();
                }
                else if (prevMode == RunMode.TELEOP_MODE && teleOpMode != null)
                {
                    teleOpMode.stop();
                }
                //
                // Start current mode.
                //
                modeStartTime = Timer.getFPGATimestamp();
                if (currMode == RunMode.DISABLED_MODE)
                {
                    LiveWindow.setEnabled(false);
                    if (disabledMode != null)
                    {
                        disabledMode.start();
                    }
                }
                else if (currMode == RunMode.TEST_MODE)
                {
                    LiveWindow.setEnabled(true);
                    if (testMode != null)
                    {
                        testMode.start();
                    }
                }
                else if (currMode == RunMode.AUTO_MODE)
                {
                    LiveWindow.setEnabled(false);
                    if (autoMode != null)
                    {
                        autoMode.start();
                    }
                }
                else if (currMode == RunMode.TELEOP_MODE)
                {
                    LiveWindow.setEnabled(false);
                    if (teleOpMode != null)
                    {
                        teleOpMode.start();
                    }
                }
                //
                // Execute all start tasks for current mode.
                //
                if (currMode != RunMode.INVALID_MODE)
                {
                    TrcTaskMgr.executeTaskType(
                            TrcTaskMgr.TaskType.START_TASK, currMode);
                }
            }

            if (nextPeriodReady())
            {
                //
                // Run periodic mode.
                //
                double timeSliceStart = Timer.getFPGATimestamp();
                TrcTaskMgr.executeTaskType(
                        TrcTaskMgr.TaskType.PREPERIODIC_TASK, currMode);
                if (currMode == RunMode.DISABLED_MODE)
                {
                    FRCNetworkCommunicationsLibrary.
                        FRCNetworkCommunicationObserveUserProgramDisabled();
                    if (disabledMode != null)
                    {
                        disabledMode.periodic();
                    }
                }
                else if (currMode == RunMode.TEST_MODE)
                {
                    FRCNetworkCommunicationsLibrary.
                        FRCNetworkCommunicationObserveUserProgramTest();
                    if (testMode != null)
                    {
                        testMode.periodic();
                    }
                }
                else if (currMode == RunMode.AUTO_MODE)
                {
                    FRCNetworkCommunicationsLibrary.
                        FRCNetworkCommunicationObserveUserProgramAutonomous();
                    if (autoMode != null)
                    {
                        autoMode.periodic();
                    }
                }
                else if (currMode == RunMode.TELEOP_MODE)
                {
                    FRCNetworkCommunicationsLibrary.
                        FRCNetworkCommunicationObserveUserProgramTeleop();
                    if (teleOpMode != null)
                    {
                        teleOpMode.periodic();
                    }
                }
                TrcTaskMgr.executeTaskType(
                        TrcTaskMgr.TaskType.POSTPERIODIC_TASK, currMode);
                double timeSliceUsed =
                        Timer.getFPGATimestamp() - timeSliceStart;
                if (debugEnabled)
                {
                    if (timeSliceUsed > timesliceThreshold)
                    {
                        dbgTrace.traceWarn(
                                funcName,
                                "%s takes too long (%5.3fs)\n",
                                currMode.toString(), timeSliceUsed);
                    }
                }
            }
            //
            // Run continuous mode.
            //
            TrcTaskMgr.executeTaskType(
                    TrcTaskMgr.TaskType.PRECONTINUOUS_TASK, currMode);
            if (currMode == RunMode.DISABLED_MODE && disabledMode != null)
            {
                disabledMode.continuous();
            }
            else if (currMode == RunMode.TEST_MODE && testMode != null)
            {
                testMode.continuous();
            }
            else if (currMode == RunMode.AUTO_MODE && autoMode != null)
            {
                autoMode.continuous();
            }
            else if (currMode == RunMode.TELEOP_MODE && teleOpMode != null)
            {
                teleOpMode.continuous();
            }
            TrcTaskMgr.executeTaskType(
                    TrcTaskMgr.TaskType.POSTCONTINUOUS_TASK, currMode);

            if (dashboardEnabled)
            {
                double elapsedTime = Timer.getFPGATimestamp() - modeStartTime;
                int elapsedMinutes = (int)(elapsedTime/60);
                elapsedTime -= elapsedMinutes*60.0;
                TrcDashboard.textPrintf(
                        0,
                        "%s: [%d:%06.3f]",
                        currMode.toString(),
                        elapsedMinutes,
                        elapsedTime);
            }
        }
    }   //startCompetition

    public void robotInit()
    {
    }   //robotInit

    private String getHostName()
    {
        String hostName = null;
        try
        {
            byte[] buff = new byte[256];
            Process proc = Runtime.getRuntime().exec("hostname");
            InputStream inStream = proc.getInputStream();
            inStream.read(buff, 0, buff.length);
            hostName = new String(buff);
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }
        return hostName;
    }   //getHostName

    /**
     * Determine if the appropriate next periodic function should be called.
     * Call the periodic functions whenever a packet is received from the
     * Driver Station or about every 20 msec.
     */
    private boolean nextPeriodReady()
    {
        return m_ds.isNewControlData();
    }   //nextPeriodReady

}   //class TrcRobot
