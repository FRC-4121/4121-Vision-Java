package frc.vision.process;

import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraBase;
import frc.vision.load.ProcessorFactory;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.stream.IntStream;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class TagAlignedColorZoneProcessor extends InstancedVisionProcessor<TagAlignedColorZoneProcessor.State> {
    private static class TaggedRect extends Rect {
        int zone = Integer.MAX_VALUE;
        public TaggedRect(Rect r) {
            super(r.x, r.y, r.width, r.height);
        }
        public TaggedRect(Rect r, int zone) {
            this(r);
            this.zone = zone;
        }
    }
    protected static class State {
        List<TaggedRect> filled;
        List<TaggedRect> empty;
    }

    public static class Position {
        // expected distance to the right
        public double x;
        // expected distance upwards
        public double y;
        // expected distance backwards
        public double z;

        // minimum area, between 0 and 1
        public double minArea = -1;
        // width of the zone
        public double w = -1;
        // height of the zone
        public double h = -1;
    }
    public static class Config extends ProcessorConfig {
        public double hmin = 0;
        public double smin = 0;
        public double vmin = 0;
        public double hmax = 255;
        public double smax = 255;
        public double vmax = 255;

        public double minArea = 1;
        // default zone width, in pixels
        public double w;
        // default zone height, in pixels
        public double h;

        // camera to get april tags from
        public String tagCam;
        // allowed april tags for us to search for
        public ArrayList<Integer> recognizedTags = new ArrayList<>();
        // available positions to search for
        public ArrayList<Position> positions = new ArrayList<>();
    }
    public static class Factory extends ProcessorFactory {
        @Override
        public String typeName() {
            return "tacz";
        }
        @Override
        public Class<Config> configType() {
            return Config.class;
        }
        @Override
        public TagAlignedColorZoneProcessor create(String name, ProcessorConfig cfg) {
            return new TagAlignedColorZoneProcessor(name, (Config)cfg);
        }
    }

    public TagAlignedColorZoneProcessor(String name, Config cfg) {
        super(name, cfg);
    }

    @Override
    public Config getConfig() {
        return (Config)super.getConfig();
    }

    @Override
    protected synchronized void processStateful(Mat img, CameraBase cam, Map<String, VisionProcessor> deps, Ref state) {
        Config cfg = getConfig();

        state.inner = new State();

        var seen = AprilTagProcessor.seen.get(cfg.tagCam == null ? cam.getName() : cfg.tagCam);
        if (seen == null) return;

        state.inner.filled = new ArrayList<>();
        state.inner.empty = new ArrayList<>();

        Mat buf = new Mat();

        for (var a : seen) {
            if (!cfg.recognizedTags.contains(a.getId())) continue;
            double sa = Math.sin(a.rotation);
            for (int i = 0; i < cfg.positions.size(); ++i) {
                Position p = cfg.positions.get(i);
                double w = p.w < 0 ? cfg.w : p.w;
                double h = p.h < 0 ? cfg.h : p.h;
                double x = a.getX() + (p.x - p.z * sa) * a.width - w * 0.5;
                double y = a.getY() + p.y * -a.height - h * 0.5;
                if (x < 0 || y < 0 || x + w >= img.cols() || y + h >= img.rows()) continue;

                Rect r = new Rect((int)x, (int)y, (int)w, (int)h);
                Mat roi = new Mat(img, r);
                Imgproc.cvtColor(roi, buf, Imgproc.COLOR_BGR2HSV);
                Core.inRange(
                    buf,
                    new Scalar(cfg.hmin, cfg.smin, cfg.vmin),
                    new Scalar(cfg.hmax, cfg.smax, cfg.vmax),
                    roi
                );

                boolean matches = Core.countNonZero(roi) / (w * h) > (p.minArea < 0 ? cfg.minArea : p.minArea);
                (matches ? state.inner.filled : state.inner.empty).add(new TaggedRect(r, a.getId() * cfg.positions.size() + i));
            }
        }
    }

    @Override
    protected void toNetworkTableStateful(NetworkTable table, Ref state) {
        if (state.inner == null) return;
        NetworkTable table_ = table.getSubTable(name);
        table_.putValue("filled", NetworkTableValue.makeIntegerArray(state.inner.filled.stream().mapToLong(x -> x.zone).toArray()));
        table_.putValue("empty", NetworkTableValue.makeIntegerArray(state.inner.empty.stream().mapToLong(x -> x.zone).toArray()));
    }

    @Override
    protected synchronized void drawOnImageStateful(Mat img, Ref state) {
        if (state.inner == null) return;
        for (var z : state.inner.filled) drawZone(img, z, z.zone, new Scalar(0, 255, 0));
        for (var z : state.inner.empty) drawZone(img, z, z.zone, new Scalar(0, 0, 255));
    }

    private static void drawZone(Mat img, Rect zone, int id, Scalar color) {
        Imgproc.rectangle(img, zone.tl(), zone.br(), color, 2);
        Imgproc.putText(
            img,
            String.valueOf(id),
            new Point(zone.x + zone.width + 5, zone.y),
            Imgproc.FONT_HERSHEY_PLAIN,
            zone.height * 0.1,
            color,
            2
        );

    }
}
