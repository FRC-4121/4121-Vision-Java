package frc.vision.process;

import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraConfig;
import java.util.IdentityHashMap;
import org.opencv.core.Mat;

// Since vision processors can be used for multiple cameras, they sometimes need to keep their states separate.
// This class maintaines an IdentityHashMap to keep the states separate for each object.
public abstract class InstancedVisionProcessor<S> extends VisionProcessor {
    // A simple wrapper around the state to allow passing by reference.
    protected class Ref {
        public S inner;
    }

    // The states for this processor.
    protected IdentityHashMap<Object, Ref> states;

    protected InstancedVisionProcessor(String name) {
        super(name);
        states = new IdentityHashMap();
    }

    // Process an image, but given a state instead of just a handle.
    protected abstract void processStateful(Mat img, CameraConfig cfg, Ref state);

    // Write data to a network table, but given a state instead of just a handle.
    protected abstract void toNetworkTableStateful(NetworkTable table, Ref state);

    // Draw on an image, but given a state instead of just a handle.
    protected abstract void drawOnImageStateful(Mat img, Ref state);

    @Override
    public void process(Mat img, CameraConfig cfg, Object handle) {
        Ref state = states.putIfAbsent(handle, new Ref());
        processStateful(img, cfg, state);
    }

    @Override
    public void toNetworkTable(NetworkTable table, Object handle) {
        Ref state = states.get(handle);
        assert state != null;
        toNetworkTableStateful(table, state);
    }

    @Override
    public void drawOnImage(Mat img, Object handle) {
        Ref state = states.get(handle);
        assert state != null;
        drawOnImageStateful(img, state);
    }
}
