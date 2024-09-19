import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.Reader;
import java.io.Writer;
import java.util.Calendar;
import java.util.Map;
import java.util.LinkedHashMap;
import java.util.concurrent.*;
import java.util.function.Supplier;
import com.google.gson.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/// Base class for all camera objects.
/// Implements functional interfaces.
abstract class CameraBase implements Runnable, Callable<Mat>, Supplier<Mat> {
    /// Name of the camera, used for loading from logs and debugging
    protected String name;
    /// Camera configuration
    protected CameraConfig config;
    /// The most recent successful frame from the camera. May be mutated by vision processors
    protected Mat frame;
    /// Writer to log debug information and exceptions
    protected PrintWriter log;

    protected static LinkedHashMap<String, CameraConfig> configs; 
    private static boolean configInitialized = false;
    public static File logDir;
    private static final String logNameFormat = "log_%s_%04%02%02_%02%02%02.txt";


    CameraBase(String name) throws FileNotFoundException {
        this(name, Calendar.getInstance());
    }
    CameraBase(String name, Calendar date) throws FileNotFoundException {
        this.name = name;
        assert configInitialized;
        this.config = configs.get(name);
        assert this.config != null;
        File path = new File(logDir, String.format(logNameFormat, name, date.get(Calendar.YEAR), date.get(Calendar.MONTH), date.get(Calendar.DATE), date.get(Calendar.HOUR), date.get(Calendar.MINUTE), date.get(Calendar.SECOND)));
        log = new PrintWriter(path);
        log.write("logging for " + name + " at " + date.toString() + "\n");
    }

    /// Initialize configs from a file, in JSON.
    public static void initConfig(Reader file) throws JsonSyntaxException, JsonIOException {
        Gson gson = new Gson();
        configs = gson.fromJson(file, LinkedHashMap.class);
        
        for (Map.Entry<String, CameraConfig> entry : configs.entrySet()) {
            if (entry.getKey() == "default") continue;
            CameraConfig default_ = configs.get("default");
            if (default_ != null) entry.getValue().updateFrom(default_);
        }
        configInitialized = true;
    }

    /// Main customization point for the camera. Read a single frame, or null if it failed.
    protected abstract Mat readFrameRaw();

    /// Method to be called after all cameras are initialized.
    protected void postInit() {}

    /// Read a frame, do basic processing
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
