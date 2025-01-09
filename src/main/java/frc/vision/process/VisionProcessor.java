package frc.vision.process;

import edu.wpi.first.networktables.NetworkTable;
import frc.vision.camera.CameraBase;
import org.opencv.core.Mat;

// Base class for vision processors
public abstract class VisionProcessor {
    String name;
    ProcessorConfig config;
    VisionProcessor(String name, ProcessorConfig config) {
        this.name = name;
        this.config = config;
    }
    public String getName() {
        return name;
    }
    public ProcessorConfig getConfig() {
        return config;
    }
    // Process the input image, given the image directly from the camera.
    // This must not modify img in any way!
    // A handle is passed so previously seen results can be reused.
    public abstract void process(Mat img, CameraBase handle);

    // Send the output to a given network table.
    // The same handle object passed to process will also be passed here.
    public abstract void toNetworkTable(NetworkTable table, CameraBase handle);

    // Draw the output to a given image, for debugging purposes.
    // The image and handle passed will be the same as the one passed to process()
    public abstract void drawOnImage(Mat img, CameraBase handle);
}
