package frc.vision.camera;

import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.util.PixelFormat;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.Duration;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.concurrent.*;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

// Base class for all camera objects.
// Implements functional interfaces.
public abstract class CameraBase implements Runnable, Callable<Mat>, Supplier<Mat> {
    // Name of the camera, used for loading from logs and debugging.
    protected String name;
    // Camera configuration.
    protected CameraConfig config;
    // The most recent successful frame from the camera. May be mutated by vision processors
    protected Mat frame;
    // Writer to log debug information and exceptions.
    protected PrintWriter log;
    // Whether we should catch exceptions
    protected boolean catchExceptions;

    protected ReentrantLock cameraLock;

    protected Instant lastFrame;

    protected CvSource source;
    protected MjpegServer sink;

    public static boolean echoErrors = false;
    public static File logDir = new File("logs/cam");
    public static final String logNameFormat = "log_%s_%s.txt";
    public static final DateTimeFormatter logDateFormat = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");

    protected CameraBase(String name, CameraConfig cfg) throws IOException {
        this(name, cfg, LocalDateTime.now());
    }
    protected CameraBase(String name, CameraConfig cfg, LocalDateTime date) throws IOException {
        this.name = name;
        this.config = cfg;
        this.catchExceptions = true;
        this.cameraLock = new ReentrantLock();
        String filename = String.format(logNameFormat, name, logDateFormat.format(date));
        File path = new File(logDir, filename);
        log = new PrintWriter(path);
        // log = new PrintWriter(System.out);
        log.write(String.format("logging for %s at %s\n", name, date));
        log.flush();
        File link = new File(logDir, String.format(logNameFormat, name, "LATEST"));
        link.delete();
        Files.createSymbolicLink(link.toPath(), Paths.get(filename));
        if (cfg.stream != null) {
            try {
                String streamName = cfg.stream.name == null ? name : cfg.stream.name;
                source = new CvSource(name + "-src", PixelFormat.kBGR, cfg.width, cfg.height, cfg.stream.fps);
                sink = cfg.stream.address == null ? new MjpegServer(streamName, cfg.stream.port) : new MjpegServer(streamName, cfg.stream.address, cfg.stream.port);
                sink.setSource(source);
            } catch (Exception e) {
                e.printStackTrace(log);
            }
        }
    }

    // Main customization point for the camera. Read a single frame, or null if it failed.
    protected abstract Mat readFrameRaw() throws Exception;

    // Method to be called after all cameras are initialized.
    public void postInit() {}

    // Return whether or not we're currently catching exceptions that occur
    public boolean getCatchExceptions() {
        return catchExceptions;
    }

    // Set whether or not exceptions are being caught. If true, any exceptions that occur when reading a frame will be logged.
    public void setCatchExceptions(boolean ex) {
        catchExceptions = ex;
    }

    // Read a frame, do basic processing
    public Mat readFrame() throws Exception {
        boolean locked = false;
        try {
            if (!cameraLock.tryLock(config.lockTimeout, TimeUnit.MILLISECONDS)) return this.frame;
            locked = true;
            Mat frame = readFrameRaw();
            if (lastFrame != null) {
                Duration dur = Duration.between(lastFrame, Instant.now());
                long toSleep = (long)(1000.0 / (float)config.fpsThrottle) - dur.toMillis();
                if (toSleep > 0) Thread.sleep(toSleep);
            }
            lastFrame = Instant.now();
            if (frame == null) return this.frame;
            // if (frame.dataAddr() == 0) return this.frame;
            if (config.enforceSize && !(frame.rows() == config.height && frame.cols() == config.width)) {
                if (this.frame == null) this.frame = new Mat();
                Imgproc.resize(frame, this.frame, new Size(config.width, config.height));
            }
            else this.frame = frame;
            drawOnFrame(frame);
            if (source != null) source.putFrame(frame);
            return frame;
        }
        catch (NullPointerException e) {
            throw e;
        }
        catch (Exception e) {
            if (catchExceptions) {
                e.printStackTrace(log);
                log.flush();
                if (echoErrors) e.printStackTrace();
            }
            else throw e;
            return frame;
        }
        finally {
            if (locked) cameraLock.unlock();
        }
    }

    private void drawOnFrame(Mat frame) {
        if (config.cropBottom > 0) Imgproc.rectangle(frame, new Point(0, frame.rows() - config.cropBottom), new Point(frame.cols(), frame.rows()), new Scalar(0));
        Scalar color = new Scalar(0, 255, 255);
        if (config.crosshair > 0) {
            int cx = frame.cols() / 2;
            int cy = frame.rows() / 2;
            Imgproc.line(frame, new Point(cx - config.crosshair, cy), new Point(cx + config.crosshair, cy), color, 2);
            Imgproc.line(frame, new Point(cx, cy - config.crosshair), new Point(cx, cy + config.crosshair), color, 2);
        } else if (config.crosshair < 0) {
            int mx = frame.cols();
            int my = frame.rows();
            int cx = mx / 2;
            int cy = my / 2;
            Imgproc.line(frame, new Point(0, cy), new Point(mx, cy), color, 2);
            Imgproc.line(frame, new Point(cx, 0), new Point(cx, my), color, 2);
            if (config.crosshair < -1) {
                int cx1 = cx / 2;
                int cx2 = cx + cx1;
                int cy1 = cy / 2;
                int cy2 = cy + cy1;
                Imgproc.line(frame, new Point(cx - 10, cy1), new Point(cx + 10, cy1), color, 1);
                Imgproc.line(frame, new Point(cx - 10, cy2), new Point(cx + 10, cy2), color, 1);
                Imgproc.line(frame, new Point(cx1, cy - 10), new Point(cx1, cy + 10), color, 1);
                Imgproc.line(frame, new Point(cx2, cy - 10), new Point(cx2, cy + 10), color, 1);
            }
        }
        if (config.bottomLeft != null) {
            Size sz = Imgproc.getTextSize(
                config.bottomLeft,
                Imgproc.FONT_HERSHEY_PLAIN,
                1,
                2,
                null
            );
            Imgproc.putText(
                frame,
                config.bottomLeft,
                new Point(5, frame.rows() - sz.height),
                Imgproc.FONT_HERSHEY_PLAIN,
                1,
                color,
                2
            );
        }
        if (config.bottomRight != null) {
            Size sz = Imgproc.getTextSize(
                config.bottomRight,
                Imgproc.FONT_HERSHEY_PLAIN,
                1,
                2,
                null
            );
            Imgproc.putText(
                frame,
                config.bottomRight,
                new Point(frame.cols() - sz.width - 5, frame.rows() - sz.height),
                Imgproc.FONT_HERSHEY_PLAIN,
                1,
                color,
                2
            );
        }
    }

    public ReentrantLock getLock() {
        return cameraLock;
    }

    public CameraConfig getConfig() {
        return config;
    }

    public String getName() {
        return name;
    }

    public Mat getFrame() {
        return frame;
    }

    public PrintWriter getLog() {
        return log;
    }

    public Mat get() {
        try {
            return readFrame();
        } catch (RuntimeException e) {
            throw e;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
    public Mat call() throws Exception {
        return readFrame();
    }
    public void run() {
        try {
            readFrame();
        } catch (RuntimeException e) {
            throw e;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
