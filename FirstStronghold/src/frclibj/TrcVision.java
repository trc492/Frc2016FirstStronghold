package frclibj;

import java.util.Comparator;
import java.util.Vector;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class TrcVision implements Runnable
{
    private static final String moduleName = "TrcVision";
    private static final boolean debugEnabled = false;
    private static final boolean visionPerfEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public class ParticleReport
            implements Comparator<ParticleReport>, Comparable<ParticleReport>
    {
        public int imageWidth;
        public int imageHeight;
        public double percentAreaToImageArea;
        public double area;
        public double boundingRectLeft;
        public double boundingRectTop;
        public double boundingRectRight;
        public double boundingRectBottom;

        public int compareTo(ParticleReport r)
        {
            return (int)(r.area - this.area);
        }

        public int compare(ParticleReport r1, ParticleReport r2)
        {
            return (int)(r1.area - r2.area);
        }
    }   //class ParticleReport

    private AxisCamera camera;
    private ColorMode colorMode;
    private Range[] colorThresholds;
    private boolean doConvexHull;
    private ParticleFilterCriteria2[] filterCriteria;
    private ParticleFilterOptions2 filterOptions;

    private Image image;
    private Image binaryImage;
    private Object monitor;
    private Thread visionThread = null;

    private String imageFile = null;
    private double processingPeriod = 0.05;
    private boolean sendImageEnabled = true;
    private boolean taskEnabled = false;
    private boolean oneShotEnabled = false;
    private Vector<ParticleReport> targets = null;

    public TrcVision(
            AxisCamera camera,
            ImageType imageType,
            ColorMode colorMode,
            Range[] colorThresholds,
            boolean doConvexHull,
            ParticleFilterCriteria2[] filterCriteria,
            ParticleFilterOptions2 filterOptions)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.camera = camera;
        this.colorMode = colorMode;
        this.colorThresholds = colorThresholds;
        this.doConvexHull = doConvexHull;
        this.filterCriteria = filterCriteria;
        this.filterOptions = filterOptions;
        if (colorThresholds.length != 3)
        {
            throw new IllegalArgumentException(
                    "Color threshold array must have 3 elements.");
        }

        image = NIVision.imaqCreateImage(imageType, 0);
        binaryImage = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);

        monitor = new Object();
        visionThread = new Thread(this, "VisionTask");
        visionThread.start();
    }   //TrcVision

    public void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "enabled=%s", Boolean.toString(enabled));
        }

        if (!taskEnabled && enabled)
        {
            //
            // Enable task.
            //
            synchronized(monitor)
            {
                taskEnabled = true;
                monitor.notify();
            }
        }
        else if (taskEnabled && !enabled)
        {
            //
            // Disable task.
            //
            taskEnabled = false;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTaskEnabled

    public boolean isTaskEnabled()
    {
        final String funcName = "isTaskEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                    "=%s", Boolean.toString(taskEnabled));
        }

        return taskEnabled;
    }   //isTaskEnabled

    public void setProcessingPeriod(double period)
    {
        final String funcName = "setProcessingPeriod";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "period=%f", period);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        processingPeriod = period;
    }   //setProcessPeriod

    public double getProcessingPeriod()
    {
        final String funcName = "getProcessingPeriod";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", processingPeriod);
        }

        return processingPeriod;
    }   //getProcessingPeriod

    public void setImageFile(String imageFile)
    {
        final String funcName = "setImageFile";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "file=%s", imageFile);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.imageFile = imageFile;
    }   //setImageFile

    public void setSendImageEnabled(boolean enabled)
    {
        final String funcName = "setSendImageEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API);
        }

        sendImageEnabled = enabled;
    }   //setSendImageEnabled

    public Vector<ParticleReport> getTargets()
    {
        final String funcName = "getTargets";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API);
        }

        Vector<ParticleReport> newTargets = null;
        synchronized(monitor)
        {
            if (!taskEnabled && targets == null)
            {
                oneShotEnabled = true;
                monitor.notify();
            }
            newTargets = targets;
            targets = null;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API);
        }
        return newTargets;
    }   //getTargets

    public void run()
    {
        while (true)
        {
            synchronized(monitor)
            {
                while (!taskEnabled && !oneShotEnabled)
                {
                    try
                    {
                        monitor.wait();
                    }
                    catch (InterruptedException e)
                    {
                    }
                }
            }

            double nextPeriodTime =
                    Timer.getFPGATimestamp() + processingPeriod;

            processImage();

            while (Timer.getFPGATimestamp() < nextPeriodTime)
            {
                try
                {
                    Thread.sleep((int)
                            ((nextPeriodTime - Timer.getFPGATimestamp())*
                             1000));
                }
                catch (InterruptedException e)
                {
                }
                catch (IllegalArgumentException e)
                {
                    break;
                }
            }
        }
    }   //run

    private void processImage()
    {
        final String funcName = "processImage";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        boolean hasImage = false;
        double totalTime = 0.0;
        double startTime;
        double deltaTime;

        if (camera != null)
        {
            hasImage = camera.isFreshImage() && camera.getImage(image);
        }
        else if (imageFile != null)
        {
            NIVision.imaqReadFile(image, imageFile);
            hasImage = true;
        }

        if (hasImage)
        {
            if (visionPerfEnabled)
            {
                startTime = Timer.getFPGATimestamp();
            }
            NIVision.imaqColorThreshold(
                    binaryImage,
                    image,
                    255,
                    colorMode,
                    colorThresholds[0],
                    colorThresholds[1],
                    colorThresholds[2]);
            if (visionPerfEnabled)
            {
                deltaTime = Timer.getFPGATimestamp() - startTime;
                totalTime += deltaTime;
                TrcDashboard.putNumber("ColorThresholdTime", deltaTime);
            }

            if (doConvexHull)
            {
                if (visionPerfEnabled)
                {
                    startTime = Timer.getFPGATimestamp();
                }
                NIVision.imaqConvexHull(binaryImage, binaryImage, 1);
                if (visionPerfEnabled)
                {
                    deltaTime = Timer.getFPGATimestamp() - startTime;
                    totalTime += deltaTime;
                    TrcDashboard.putNumber("ConvexHullTime", deltaTime);
                }
            }

            if (visionPerfEnabled)
            {
                startTime = Timer.getFPGATimestamp();
            }
            NIVision.imaqParticleFilter4(
                    binaryImage,
                    binaryImage,
                    filterCriteria,
                    filterOptions,
                    null);
            if (visionPerfEnabled)
            {
                deltaTime = Timer.getFPGATimestamp() - startTime;
                totalTime += deltaTime;
                TrcDashboard.putNumber("ParticleFilterTime", deltaTime);
            }

            int numParticles = NIVision.imaqCountParticles(binaryImage, 1);
            if(numParticles > 0)
            {
                //
                // Measure particles and sort by particle size.
                //
                Vector<ParticleReport> particles = new Vector<ParticleReport>();
                GetImageSizeResult imageSize =
                        NIVision.imaqGetImageSize(binaryImage);
                if (visionPerfEnabled)
                {
                    startTime = Timer.getFPGATimestamp();
                }

                for(int i = 0; i < numParticles; i++)
                {
                    ParticleReport par = new ParticleReport();
                    par.imageWidth = imageSize.width;
                    par.imageHeight = imageSize.height;
                    par.percentAreaToImageArea =
                            NIVision.imaqMeasureParticle(
                                    binaryImage,
                                    i,
                                    0,
                                    MeasurementType.MT_AREA_BY_IMAGE_AREA);
                    par.area =
                            NIVision.imaqMeasureParticle(
                                    binaryImage,
                                    i,
                                    0,
                                    MeasurementType.MT_AREA);
                    par.boundingRectTop =
                            NIVision.imaqMeasureParticle(
                                    binaryImage,
                                    i,
                                    0,
                                    MeasurementType.MT_BOUNDING_RECT_TOP);
                    par.boundingRectLeft =
                            NIVision.imaqMeasureParticle(
                                    binaryImage,
                                    i,
                                    0,
                                    MeasurementType.MT_BOUNDING_RECT_LEFT);
                    par.boundingRectBottom =
                            NIVision.imaqMeasureParticle(
                                    binaryImage,
                                    i,
                                    0,
                                    MeasurementType.MT_BOUNDING_RECT_BOTTOM);
                    par.boundingRectRight =
                            NIVision.imaqMeasureParticle(
                                    binaryImage,
                                    i,
                                    0,
                                    MeasurementType.MT_BOUNDING_RECT_RIGHT);
                    particles.add(par);
                }
                particles.sort(null);
                if (visionPerfEnabled)
                {
                    deltaTime = Timer.getFPGATimestamp() - startTime;
                    totalTime += deltaTime;
                    TrcDashboard.putNumber("PrepareReportTime", deltaTime);
                }

                if (sendImageEnabled)
                {
                    CameraServer.getInstance().setImage(binaryImage);
                }

                synchronized(monitor)
                {
                    oneShotEnabled = false;
                    targets = particles;
                    particles = null;
                }
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //processImage

}   //class TrcVision
