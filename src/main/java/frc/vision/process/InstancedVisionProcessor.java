package frc.vision.process;

import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraBase;
import java.util.concurrent.ConcurrentHashMap;
import org.opencv.core.Mat;

// Since vision processors can be used for multiple cameras, they sometimes need to keep their states separate.
// This class maintaines an ConcurrentHashMap to keep the states separate for each object.
public abstract class InstancedVisionProcessor<S> extends VisionProcessor {
    // A simple wrapper around the state to allow passing by reference.
    protected class Ref {
        public S inner;
    }

    // The states for this processor.
    protected ConcurrentHashMap<CameraBase, Ref> states;

    protected InstancedVisionProcessor(String name) {
        super(name);
        states = new ConcurrentHashMap<CameraBase, Ref>();
    }

    // Process an image, but given a state instead of just a handle.
    protected abstract void processStateful(Mat img, CameraBase cam, Ref state);

    // Write data to a network table, but given a state instead of just a handle.
    protected abstract void toNetworkTableStateful(NetworkTable table, Ref state);

    // Draw on an image, but given a state instead of just a handle.
    protected abstract void drawOnImageStateful(Mat img, Ref state);

    @Override
    public void process(Mat img, CameraBase handle) {
        Ref state = states.putIfAbsent(handle, new Ref());
        if (state == null) state = states.get(handle);
        processStateful(img, handle, state);
    }

    @Override
    public void toNetworkTable(NetworkTable table, CameraBase handle) {
        Ref state = states.get(handle);
        toNetworkTableStateful(table, state);
    }

    @Override
    public void drawOnImage(Mat img, CameraBase handle) {
        Ref state = states.get(handle);
        drawOnImageStateful(img, state);
    }
}
