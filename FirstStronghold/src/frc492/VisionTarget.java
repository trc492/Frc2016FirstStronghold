package frc492;

import java.util.Vector;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;

import edu.wpi.first.wpilibj.Relay;
import frclib.FrcVision;
import frclib.FrcVision.ParticleReport;
import trclib.TrcDbgTrace;
import hallib.HalDashboard;

public class VisionTarget
{
    private static final String moduleName = "VisionTarget";
    private static final boolean debugEnabled = true;
    private TrcDbgTrace dbgTrace = null;

    private static final boolean debugVision = true;
    private HalDashboard dashboard = HalDashboard.getInstance();

    public class TargetReport
    {
        int imageWidth;
        int imageHeight;
        NIVision.Rect rect;
        double distance;

        public String toString()
        {
            return  "\nimageWidth  = " + imageWidth +
                    "\nimageHeight = " + imageHeight +
                    "\nrectLeft    = " + rect.left +
                    "\nrectTop     = " + rect.top +
                    "\nrectRight   = " + rect.left + rect.width +
                    "\nrectBottom  = " + rect.top + rect.height +
                    "\ndistance    = " + distance;
        }   //toString

    }   //class TargetReport

    private final double AREA_MINIMUM = 0.5;    //Default min area as % total area
    private final double VIEW_ANGLE = 64.0;     //View angle for Axis M1013

    private Relay ringLightPower;
    private Range[] colorThresholds;
    private ParticleFilterCriteria2[] filterCriteria;
    private ParticleFilterOptions2 filterOptions;
    private TargetReport targetReport;
    private FrcVision visionTask;

    public VisionTarget(FrcVision.ImageProvider imageProvider)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        ringLightPower = new Relay(
                RobotInfo.RELAY_RINGLIGHT_POWER, Relay.Direction.kForward);
        colorThresholds = new Range[3];
        colorThresholds[0] = new Range(101, 64);
        colorThresholds[1] = new Range(88, 255);
        colorThresholds[2] = new Range(134, 255);
        filterCriteria = new ParticleFilterCriteria2[1];
        filterCriteria[0] = new ParticleFilterCriteria2(
                MeasurementType.MT_AREA_BY_IMAGE_AREA,
                AREA_MINIMUM, 100.0,
                0, 0);
        filterOptions = new ParticleFilterOptions2(0, 0, 1, 1);
        targetReport = new TargetReport();
        targetReport.rect = new NIVision.Rect();
        visionTask = new FrcVision(
                imageProvider,
                ImageType.IMAGE_RGB,
                ColorMode.HSV,
                colorThresholds,
                false,
                filterCriteria,
                filterOptions);
    }   //VisionTarget

    public void setVisionTaskEnabled(boolean enabled)
    {
        final String funcName = "setVisionTaskEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API);
        }
        setRingLightPowerOn(enabled);
        visionTask.setTaskEnabled(enabled);
    }   //setVisionTaskEnabled

    public void setRingLightPowerOn(boolean on)
    {
        final String funcName = "setRightLightPowerOn";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "on=%s", Boolean.toString(on));
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API);
        }
        ringLightPower.set(on? Relay.Value.kOn: Relay.Value.kOff);
    }   //setRightLightPowerOn

    public TargetReport getTargetReport()
    {
        final String funcName = "getTargetReport";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        Vector<ParticleReport> reports = visionTask.getTargets();
        boolean hasReport = reports != null && reports.size() >= 1;

        if (debugEnabled)
        {
            dbgTrace.traceInfo(
                    funcName, "reports=%s,numReports=%d",
                    reports != null? "Reports": "null",
                    reports != null? reports.size(): 0);
        }

        if (hasReport)
        {
            if (debugEnabled)
            {
                for (int i = 0; i < reports.size(); i++)
                {
                        dbgTrace.traceInfo(
                                funcName,
                                "rect[%d] = {%3.0f,%3.0f/%3.0f,%3.0f}",
                                i,
                                reports.elementAt(i).boundingRectLeft,
                                reports.elementAt(i).boundingRectTop,
                                reports.elementAt(i).boundingRectRight,
                                reports.elementAt(i).boundingRectBottom);
                }
            }

            targetReport.imageWidth =
                    reports.elementAt(0).imageWidth;
            targetReport.imageHeight =
                    reports.elementAt(0).imageHeight;
            targetReport.rect.left = (int)reports.elementAt(0).boundingRectLeft;
            targetReport.rect.width = (int)(reports.elementAt(0).boundingRectRight -
                                            reports.elementAt(0).boundingRectLeft);
            targetReport.rect.top = (int)reports.elementAt(0).boundingRectTop;
            targetReport.rect.height = (int)(reports.elementAt(0).boundingRectBottom -
                                             reports.elementAt(0).boundingRectTop);
            targetReport.distance =
                    computeDistance(targetReport.rect.width, targetReport.imageWidth);

            if (debugVision)
            {
                dashboard.displayPrintf(
                        8, "imageWidth=%d, imageHeight=%d",
                        targetReport.imageWidth,
                        targetReport.imageHeight);
                dashboard.displayPrintf(
                        9, "rectLeft=%d, rectRight=%d",
                        targetReport.rect.left,
                        targetReport.rect.left + targetReport.rect.width);
                dashboard.displayPrintf(
                        10, "rectTop=%d, rectBottom=%d",
                        targetReport.rect.top,
                        targetReport.rect.top + targetReport.rect.height);
                dashboard.displayPrintf(
                        11, "dist=%f, deltaX/Y=%4.1f/%4.1f",
                        targetReport.distance,
                        (targetReport.rect.left + targetReport.rect.width/2.0 -
                         targetReport.imageWidth)/2.0,
                        (targetReport.rect.top + targetReport.rect.height/2.0 -
                         targetReport.imageHeight)/2.0);
            }

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, targetReport.toString());
            }
        }

        return hasReport? targetReport: null;
    }   //getTargetReport

    public NIVision.Rect getLastTargetRect()
    {
        return targetReport.rect;
    }   //getLastTargetRect

    /**
     * Computes the estimated distance to a target using the width of the
     * particle in the image. For more information and graphics showing the
     * math behind this approach see the Vision Processing section of the
     * ScreenStepsLive documentation.
     *
     * @param report The Particle Analysis Report for the particle
     * @param isLong Boolean indicating if the target is believed to be the
     *        long side of a tote
     * @return The estimated distance to the target in feet.
     */
    private double computeDistance (double rectWidth, double imageWidth)
    {
        double normalizedWidth = 2*(rectWidth)/imageWidth;
        double targetWidth = 15.1319;

        return targetWidth/
               (normalizedWidth*Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
    }   //computeDistance

}   //class VisionTarget
