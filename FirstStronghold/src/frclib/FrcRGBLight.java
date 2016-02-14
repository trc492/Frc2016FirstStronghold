package frclib;

import trclib.TrcDbgTrace;
import trclib.TrcEvent;

public class FrcRGBLight extends FrcPneumatic
{
    private static final String moduleName = "FrcRGBLight";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public enum RGBColor
    {
        RGB_BLACK((byte)0x0),
        RGB_RED((byte)0x1),
        RGB_GREEN((byte)0x2),
        RGB_YELLOW((byte)0x3),
        RGB_BLUE((byte)0x4),
        RGB_MAGENTA((byte)0x5),
        RGB_CYAN((byte)0x6),
        RGB_WHITE((byte)0x7);

        private byte value;

        RGBColor(byte value)
        {
            this.value = value;
        }   //RGBColor

        public byte getValue()
        {
            return value;
        }   //getValue

    }   //enum RGBColor

    public class ColorAction extends SolenoidAction
    {
        //
        // This is just for a name change.
        //
    }   //class ColorAction

    public class ColorLightState
    {
        private RGBColor color;
        private double onPeriod;
        private double offPeriod;
        private TrcEvent notifyEvent;
    }   //ColorLightState

    private ColorLightState colorLightState;

    public FrcRGBLight(
            final String instanceName,
            final int module,
            final int channel1,
            final int channel2,
            final int channel3)
    {
        super(moduleName + "." + instanceName,
              module, channel1, channel2, channel3);

        colorLightState = new ColorLightState();
        colorLightState.color = RGBColor.RGB_BLACK;
        colorLightState.onPeriod = 0.0;
        colorLightState.offPeriod = 0.0;
        colorLightState.notifyEvent = null;

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }
    }   //FrcRGBLight

    public ColorLightState getColorLightState()
    {
        final String funcName = "getColorLightState";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=(color=%s,onPeriod=%f,offPeriod=%f,event=%s)",
                    colorLightState.color.toString(),
                    colorLightState.onPeriod,
                    colorLightState.offPeriod,
                    colorLightState.notifyEvent.toString());
        }
        ColorLightState state = new ColorLightState();
        state.color = colorLightState.color;
        state.onPeriod = colorLightState.onPeriod;
        state.offPeriod = colorLightState.offPeriod;
        state.notifyEvent = colorLightState.notifyEvent;
        return state;
    }   //getColorLightState

    public void setColorLightState(ColorLightState state)
    {
        final String funcName = "setColorLightState";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "color=%s,onPeriod=%f,offPeriod=%f,event=%s",
                    colorLightState.color.toString(),
                    colorLightState.onPeriod,
                    colorLightState.offPeriod,
                    colorLightState.notifyEvent.toString());
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
        colorLightState.color = state.color;
        colorLightState.onPeriod = state.onPeriod;
        colorLightState.offPeriod = state.offPeriod;
        colorLightState.notifyEvent = state.notifyEvent;
        if (state.offPeriod != 0.0)
        {
            setColor(
                    state.color,
                    state.onPeriod,
                    state.offPeriod,
                    state.notifyEvent);
        }
        else
        {
            setColor(state.color);
        }
    }   //setColorLightState

    public RGBColor getColor()
    {
        final String funcName = "getColor";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%s", colorLightState.color.toString());
        }
        return colorLightState.color;
    }   //getColor

    public void setColor(RGBColor color)
    {
        final String funcName = "setColor";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "color=%s", color.toString());
        }

        byte bitMask = color.getValue();
        set((byte)(~bitMask), false);
        set(bitMask, true);
        colorLightState.color = color;
        colorLightState.onPeriod = 0.0;
        colorLightState.offPeriod = 0.0;
        colorLightState.notifyEvent = null;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setColor

    public void setColor(RGBColor color, double period, TrcEvent event)
    {
        final String funcName = "setColor";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "color=%s,period=%f,event=%s",
                    color.toString(), period, event.toString());
        }

        set(color.getValue(), period, event);
        //
        // One-shot color action is not resumable. Once interrupted,
        // it will just end.
        //
        colorLightState.color = RGBColor.RGB_BLACK;
        colorLightState.onPeriod = 0.0;
        colorLightState.offPeriod = 0.0;
        colorLightState.notifyEvent = null;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setColor

    public void setColor(
            RGBColor color,
            double onPeriod,
            double offPeriod,
            TrcEvent event)
    {
        final String funcName = "setColor";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "color=%s,OnPeriod=%f,OffPeriod=%f,event=%s",
                    color.toString(), onPeriod, offPeriod, event.toString());
        }

        set(color.getValue(), onPeriod, (byte)0x0, offPeriod, true, event);
        colorLightState.color = color;
        colorLightState.onPeriod = onPeriod;
        colorLightState.offPeriod = offPeriod;
        colorLightState.notifyEvent = event;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setColor

    public void setColor(
            ColorAction[] colorList,
            int numActions,
            boolean repeat,
            TrcEvent event)
    {
        final String funcName = "setColor";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "numActions=%d,repeat=%s,event=%s",
                    numActions, Boolean.toString(repeat), event.toString());
        }

        //
        // Color list action is not resumable. Once interrupted, it will just
        // end the sequence.
        //
        set(colorList, numActions, repeat, event);
        colorLightState.color = RGBColor.RGB_BLACK;
        colorLightState.onPeriod = 0.0;
        colorLightState.offPeriod = 0.0;
        colorLightState.notifyEvent = null;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setColor

}   //class FrcRGBLight
