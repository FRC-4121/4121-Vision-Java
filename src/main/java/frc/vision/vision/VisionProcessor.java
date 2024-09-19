import edu.wpi.first.networktables.NetworkTable;
import org.opencv.core.Mat;

/// Base class for vision processors
abstract class VisionProcessor {
    String name;
    VisionProcessor(String name) {
        this.name = name;
    }
    public String getName() {
        return name;
    }
    /// Process the input image, given the image directly from the camera
    public abstract void process(Mat img, CameraConfig cfg);
    /// Send the output to a given network table
    public abstract void toNetworkTable(NetworkTable table);
    /// Draw the output to a given image, for debugging purposes
    /// The image passed will be the same as the one passed to process()
    public abstract void drawOnImage(Mat img);
}
