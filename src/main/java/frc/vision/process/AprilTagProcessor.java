package frc.vision.process;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraBase;
import frc.vision.camera.CameraConfig;
import frc.vision.load.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class AprilTagProcessor extends ObjectVisionProcessor {
    protected AprilTagDetector detector;
    protected Scalar tagColor;

    public class AprilTag extends VisionObject {
        public AprilTagDetection found;
        public Transform3d pose;

        public AprilTag(AprilTagDetection found, Transform3d pose) {
            super(rectFromTag(found));
            this.found = found;
            this.pose = pose;
        }

        private static Rect rectFromTag(AprilTagDetection found) {
            double minX = found.getCornerX(0);
            double minY = found.getCornerY(0);
            double maxX = minX;
            double maxY = minY;

            for (int i = 1; i < 4; ++i) {
                double x = found.getCornerX(i);
                double y = found.getCornerY(i);

                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
            
            return new Rect(new Point(minX, minY), new Point(maxX, maxY));
        }
    
        @Override
        public void calcAngles(CameraConfig cfg) {
            if (pose == null) super.calcAngles(cfg);
            else {
                Translation3d trans = pose.getTranslation();
                double x = trans.getX();
                double y = trans.getY();
                double z = trans.getZ();
                azimuth = Math.atan2(x, z);
                elevation = -Math.atan2(y, z);
                distance = Math.sqrt(x * x + y * y + z * z);
                hasAngles = true;
            }
        }
    }

    public static ConcurrentHashMap<String, Collection<AprilTag>> seen = new ConcurrentHashMap<>();

    public AprilTagProcessor(String name, ProcessorConfig cfg, Scalar rectColor, Scalar tagColor, AprilTagDetector detector) {
        super(name, cfg, rectColor);
        this.tagColor = tagColor;
        this.detector = detector;
        this.calcAngles = true;
    }

    public AprilTagProcessor(String name, ProcessorConfig cfg, Scalar rectColor, Scalar tagColor, String family) {
        this(name, cfg, rectColor, tagColor);
        detector.addFamily(family);
    }

    public AprilTagProcessor(String name, ProcessorConfig cfg, Scalar rectColor, Scalar tagColor) {
        this(name, cfg, rectColor, tagColor, new AprilTagDetector());
    }

    public AprilTagProcessor(String name, ProcessorConfig cfg, Scalar rectColor, String family) {
        this(
            name,
            cfg,
            rectColor,
            new Scalar(
                255 - rectColor.val[0],
                255 - rectColor.val[1],
                255 - rectColor.val[2]
            ),
            family
        );
    }

    public AprilTagProcessor(String name, ProcessorConfig cfg, Scalar rectColor) {
        this(
            name,
            cfg,
            rectColor,
            new Scalar(
                255 - rectColor.val[0],
                255 - rectColor.val[1],
                255 - rectColor.val[2]
            ),
            defaultDetector()
        );
    }

    public AprilTagProcessor(String name, ProcessorConfig cfg, String family) {
        this(name, cfg, new Scalar(0, 0, 255), family);
    }

    public AprilTagProcessor(String name, ProcessorConfig cfg) {
        this(name, cfg, new Scalar(0, 0, 255));
    }

    public AprilTagDetector getDetector() {
        return detector;
    }

    public static AprilTagDetector defaultDetector() {
        AprilTagDetector detect = new AprilTagDetector();
        AprilTagDetector.Config cfg = new AprilTagDetector.Config();
        detect.setConfig(cfg);
        detect.addFamily("tag36h11");
        return detect;
    }

    @Override
    protected Collection<VisionObject> processObjects(Mat img, CameraBase cam, Map<String, VisionProcessor> _deps) {
        AprilTagDetection[] tags = new AprilTagDetection[0];
        Mat grayFrame = new Mat();
        switch (img.channels()) {
            case 1:
                img.copyTo(grayFrame);
                break;
            case 3:
                Imgproc.cvtColor(img, grayFrame, Imgproc.COLOR_BGR2GRAY);
                break;
            default:
                throw new RuntimeException("Wrong number of image channels!");
        }
        synchronized(detector) {
            tags = detector.detect(grayFrame);
        }
        // unlike the detector, the estimator seems to just be a wrapper around the config
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(cam.getConfig().poseConfig());
        Collection<AprilTag> tagCollection = Arrays.stream(tags)
            .map(obj -> new AprilTag(obj, estimator.estimate(obj)))
            .collect(Collectors.toList());
        seen.put(cam.getName(), tagCollection);
        return tagCollection.stream().collect(Collectors.toList());
    }

    @Override
    public void toNetworkTableStateful(NetworkTable table, Ref state) {
        super.toNetworkTableStateful(table, state);
        NetworkTable table_ = table.getSubTable(name);
        int size = state.inner.size();
        long[] ids = new long[size];
        int i = 0;
        for (VisionObject obj : state.inner) {
            AprilTag obj_ = (AprilTag)obj;
            AprilTagDetection tag = obj_.found;
            ids[i] = tag.getId();
            ++i;
        }
        table_.putValue("ids", NetworkTableValue.makeIntegerArray(ids));
    }

    @Override
    public void drawOnImageStateful(Mat img, Ref state) {
        super.drawOnImageStateful(img, state);
        for (VisionObject obj : state.inner) {
            AprilTagDetection tag = ((AprilTag)obj).found;
            Imgproc.polylines(
                img,
                Arrays.asList(new MatOfPoint(
                    new Point(tag.getCornerX(0), tag.getCornerY(0)),
                    new Point(tag.getCornerX(1), tag.getCornerY(1)),
                    new Point(tag.getCornerX(2), tag.getCornerY(2)),
                    new Point(tag.getCornerX(3), tag.getCornerY(3))
                )),
                true,
                tagColor,
                5,
                -1
            );
            double h = ((AprilTag)obj).height;
            h *= 0.1;
            Imgproc.putText(
                img,
                String.valueOf(tag.getId()),
                new Point(tag.getCenterX() - h, tag.getCenterY()),
                Imgproc.FONT_HERSHEY_PLAIN,
                h * 0.25,
                tagColor,
                2
            );
        }
    }

    public static class Config extends ProcessorConfig {
        public ArrayList<String> family;
    }
    public static class Factory extends ProcessorFactory {
        @Override
        public String typeName() {
            return "apriltag";
        }
        @Override
        public Class<Config> configType() {
            return Config.class;
        }
        @Override
        public AprilTagProcessor create(String name, ProcessorConfig cfg) {
            AprilTagProcessor out = new AprilTagProcessor(name, cfg);
            Config cfg_ = (Config)cfg;
            if (cfg_.family != null) {
                for (String family : cfg_.family) {
                    out.getDetector().addFamily(family);
                }
            }
            return out;
        }
    }
}
