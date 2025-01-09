package frc.vision.camera;

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
            Imgproc.rectangle(frame, new Point(0, frame.rows() - config.cropBottom), new Point(frame.cols(), frame.rows()), new Scalar(0));
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
