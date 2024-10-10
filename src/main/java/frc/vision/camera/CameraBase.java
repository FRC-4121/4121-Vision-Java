package frc.vision.camera;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Writer;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.Duration;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.concurrent.*;
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

    protected Instant lastFrame;

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
        String filename = String.format(logNameFormat, name, logDateFormat.format(date));
        File path = new File(logDir, filename);
        log = new PrintWriter(path);
        log.write(String.format("logging for %s at %s\n", name, date));
        log.flush();
        File link = new File(logDir, String.format(logNameFormat, name, "LATEST"));
        link.delete();
        Files.createSymbolicLink(link.toPath(), Paths.get(filename));
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
        try {
            Mat frame = readFrameRaw();
            if (lastFrame != null) {
                Duration dur = Duration.between(lastFrame, Instant.now());
                long toSleep = (long)(1000.0 / (float)config.fpsThrottle) - dur.toMillis();
                if (toSleep > 0) Thread.currentThread().sleep(toSleep);
            }
            lastFrame = Instant.now();
            if (frame == null) return this.frame;
            this.frame = frame;
            Imgproc.rectangle(frame, new Point(0, config.height - config.cropBottom), new Point(config.width, config.height), new Scalar(0));
            return frame;
        } 
        catch (NullPointerException e) {
            throw e;
        }
        catch (Exception e) {
            if (catchExceptions) e.printStackTrace(log);
            else throw e;
            return frame;
        }
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
