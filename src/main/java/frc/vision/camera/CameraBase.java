package frc.vision.camera;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.Writer;
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

    public static File logDir = new File("logs/cam");
    protected static final String logNameFormat = "log_%s_%s.txt";
    protected static final DateTimeFormatter logDateFormat = DateTimeFormatter.ofPattern("yyyyMMdd_HHMMSS");


    protected CameraBase(String name, CameraConfig cfg) throws FileNotFoundException {
        this(name, cfg, LocalDateTime.now());
    }
    protected CameraBase(String name, CameraConfig cfg, LocalDateTime date) throws FileNotFoundException {
        this.name = name;
        this.config = cfg;
        File path = new File(logDir, String.format(logNameFormat, name, logDateFormat.format(date)));
        log = new PrintWriter(path);
        log.write("logging for " + name + " at " + date.toString() + "\n");
    }

    // Main customization point for the camera. Read a single frame, or null if it failed.
    protected abstract Mat readFrameRaw();

    // Method to be called after all cameras are initialized.
    protected void postInit() {}

    // Read a frame, do basic processing
    public Mat readFrame() {
        try {
            Mat frame = readFrameRaw();
            if (frame == null) return this.frame;
            this.frame = frame;
            Imgproc.rectangle(frame, new Point(0, config.height - config.cropBottom), new Point(config.width, config.height), new Scalar(0));
            return frame;
        } catch (Exception e) {
            e.printStackTrace(log);
            return null;
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

    public Mat get() {
        return readFrame();
    }
    public Mat call() {
        return readFrame();
    }
    public void run() {
        readFrame();
    }
}
