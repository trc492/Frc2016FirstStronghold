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

package trclib;

import trclib.TrcTaskMgr.TaskType;

public abstract class TrcRGBLight implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcRGBLight";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public abstract boolean getRed();
    public abstract boolean getGreen();
    public abstract boolean getBlue();
    public abstract void setRed(boolean enabled);
    public abstract void setGreen(boolean enabled);
    public abstract void setBlue(boolean enabled);

    public static final int COLOR_BLACK     = 0;
    public static final int COLOR_RED       = (1 << 0);
    public static final int COLOR_GREEN     = (1 << 1);
    public static final int COLOR_BLUE      = (1 << 2);
    public static final int COLOR_YELLOW    = (COLOR_RED | COLOR_GREEN);
    public static final int COLOR_MAGENTA   = (COLOR_RED | COLOR_BLUE);
    public static final int COLOR_CYAN      = (COLOR_GREEN | COLOR_BLUE);
    public static final int COLOR_WHITE     = (COLOR_RED | COLOR_GREEN | COLOR_BLUE);
    public static final int COLOR_MIN_VALUE = COLOR_BLACK;
    public static final int COLOR_MAX_VALUE = COLOR_WHITE;

    public enum RGBColor
    {
        RGB_INVALID(-1),
        RGB_BLACK(COLOR_BLACK),
        RGB_RED(COLOR_RED),
        RGB_GREEN(COLOR_GREEN),
        RGB_YELLOW(COLOR_YELLOW),
        RGB_BLUE(COLOR_BLUE),
        RGB_MAGENTA(COLOR_MAGENTA),
        RGB_CYAN(COLOR_CYAN),
        RGB_WHITE(COLOR_WHITE);

        private int value;

        private RGBColor(int value)
        {
            this.value = value;
        }   //RGBColor

        public int getValue()
        {
            return value;
        }   //getValue

        public static RGBColor getColor(int value)
        {
            for (RGBColor color: RGBColor.values())
            {
                if (value == color.getValue())
                {
                    return color;
                }
            }

            return RGB_INVALID;
        }   //getColor

    }   //enum RGBColor

    private enum State
    {
        TURN_ON,
        TURN_OFF,
        DONE
    }   //enum State

    private final String instanceName;
    private TrcStateMachine sm;
    private TrcTimer timer;
    private TrcEvent timerEvent;

    private RGBColor color = RGBColor.RGB_BLACK;
    private double onPeriod = 0.0;
    private double offPeriod = 0.0;
    private TrcEvent notifyEvent = null;

    public TrcRGBLight(String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.instanceName = instanceName;
        sm = new TrcStateMachine(moduleName);
        timer = new TrcTimer(moduleName);
        timerEvent = new TrcEvent(moduleName + ".timer");
    }   //TrcRGBLight

    public String toString()
    {
        return instanceName;
    }   //toString

    private void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.FUNC,
                    "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }

        if (enabled)
        {
            TrcTaskMgr.getInstance().registerTask(moduleName, this, TaskType.POSTCONTINUOUS_TASK);
        }
        else
        {
            TrcTaskMgr.getInstance().unregisterTask(this, TaskType.POSTCONTINUOUS_TASK);
        }
    }   //setTaskEnabled

    private int getColorValue()
    {
        final String funcName = "getColorValue";
        int colorValue = 0;

        if (getRed()) colorValue |= COLOR_RED;
        if (getGreen()) colorValue |= COLOR_GREEN;
        if (getBlue()) colorValue |= COLOR_BLUE;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=0xx", colorValue);
        }

        return colorValue;
    }   //getColorValue

    public RGBColor getColor()
    {
        final String funcName = "getColor";
        RGBColor color;
 
        color = RGBColor.getColor(getColorValue());

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", color.toString());
        }

        return color;
    }   //getColor

    private void setColorValue(int colorValue)
    {
        final String funcName = "setColorValue";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "color=0x%x", colorValue);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setRed((colorValue & COLOR_RED) != 0);
        setGreen((colorValue & COLOR_GREEN) != 0);
        setBlue((colorValue & COLOR_BLUE) != 0);
    }   //setColorValue

    public void setColor(RGBColor color)
    {
        final String funcName = "setColorValue";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "color=%s", color.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (sm.isEnabled())
        {
            timer.cancel();
            sm.stop();
        }

        setColorValue(color.getValue());
    }   //setColor

    public void setColor(RGBColor color, double onPeriod, double offPeriod)
    {
        final String funcName = "setColor";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "color=%s,onPeriod=%f,offPeriod=%f",
                    color.toString(), onPeriod, offPeriod);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        setColor(RGBColor.RGB_BLACK);
        this.color = color;
        this.onPeriod = onPeriod;
        this.offPeriod = offPeriod;
        this.notifyEvent = null;
        sm.start(State.TURN_ON);
        setTaskEnabled(true);
    }   //setColor

    public void setColor(RGBColor color, double onPeriod, TrcEvent event)
    {
        setColor(color, onPeriod, 0.0);
        this.notifyEvent = event;
    }   //setColor

    public void setColor(RGBColor color, double onPeriod)
    {
        setColor(color, onPeriod, 0.0);
    }   //setColor

    //
    // Implements TrcTaskMgr.Task
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
        setColor(RGBColor.RGB_BLACK);
    }   //stopTask

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //prePeriodicTask

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "postContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        if (sm.isReady())
        {
            State state = (State)sm.getState();

            switch (state)
            {
                case TURN_ON:
                    setColorValue(color.getValue());
                    if (onPeriod != 0.0)
                    {
                        timer.set(onPeriod, timerEvent);
                        sm.addEvent(timerEvent);
                        sm.waitForEvents(State.TURN_OFF);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case TURN_OFF:
                    setColorValue(COLOR_BLACK);
                    if (offPeriod != 0.0)
                    {
                        timer.set(offPeriod, timerEvent);
                        sm.addEvent(timerEvent);
                        sm.waitForEvents(State.TURN_ON);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                    if (notifyEvent != null)
                    {
                        notifyEvent.set(true);
                    }
                    sm.stop();
                    setTaskEnabled(false);
                    break;
            }
        }
    }   //postContinuousTask

}   //class TrcRGBLight
