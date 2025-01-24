package frc.vision.process;

import org.opencv.core.*;


public class Coral2025Processor extends InstancedVisionProcessor<Coral2025Processor.State> {
    public static class State {

    }

    public static class Position {
        // expected distance to the right
        public double x;
        // expected distance upwards
        public double y;
        // expected distance backwards
        public double z;
        // expected angle, rotated toward the camera
        public double angle;
    }
    public static class Config extends RectVisionProcessor.Config {
        // initial width, in april tag widths
        public double iw = Double.INFINITY;
        // initial height, in april tag heights
        public double ih = Double.INFINITY;
        // initial distance to the right, in april tag widths
        public double idx = 0;
        // initial distance upwards, in april tag heights
        public double idy = 0;
        // initial distance between the coral plane and the tag plane, in april tag widths
        public double idz = 0;
        // maximum distance between the measured and expected center
        public double dmax = Double.INFINITY;
        // maximum aspect ratio difference
        public double amax = Double.INFINITY;
        // maximum rotation difference
        public double rmax = Math.PI;

        // camera to get april tags from
        public String tagCam;
        // allowed april tags for us to search for
        public ArrayList<Integer> recognizedTags = new ArrayList<>();
        // available positions to search for
        public ArrayList<Position> positions = new ArrayList<>();
    }

    public Coral2025Processor(String name, Config cfg) {
        super(name, cfg);
    }

    @Override
    public Config getConfig() {
        return (Config)super.getConfig();
    }

    protected void processStateful(Mat img, CameraBase cam, Ref state) {}

    protected void toNetworkTableStateful(NetworkTable table, Ref state) {}

    protected void drawOnImageStateful(Mat img, Ref state) {}
}
