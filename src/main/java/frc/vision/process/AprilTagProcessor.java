package frc.vision.process;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.networktables.*;
import frc.vision.camera.CameraConfig;
import frc.vision.load.*;
import java.time.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.stream.Collectors;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class AprilTagProcessor extends ObjectVisionProcessor {
    protected AprilTagDetector detector;
    protected Scalar tagColor;

    public static class AprilTag extends VisionObject {
        public AprilTagDetection found;

        public AprilTag(AprilTagDetection found) {
            super(rectFromTag(found));
            this.found = found;
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
    }

    public AprilTagProcessor(String name, Scalar rectColor, Scalar tagColor, AprilTagDetector detector) {
        super(name, rectColor);
        this.tagColor = tagColor;
        this.detector = detector;
    }

    public AprilTagProcessor(String name, Scalar rectColor, Scalar tagColor, String family) {
        this(name, rectColor, tagColor);
        detector.addFamily(family);
    }

    public AprilTagProcessor(String name, Scalar rectColor, Scalar tagColor) {
        this(name, rectColor, tagColor, new AprilTagDetector());
    }

    public AprilTagProcessor(String name, Scalar rectColor, String family) {
        this(
            name,
            rectColor,
            new Scalar(
                255 - rectColor.val[0],
                255 - rectColor.val[1],
                255 - rectColor.val[2]
            ),
            family
        );
    }

    public AprilTagProcessor(String name, Scalar rectColor) {
        this(
            name,
            rectColor,
            new Scalar(
                255 - rectColor.val[0],
                255 - rectColor.val[1],
                255 - rectColor.val[2]
            ),
            defaultDetector()
        );
    }

    public AprilTagProcessor(String name, String family) {
        this(name, new Scalar(0, 0, 255), family);
    }

    public AprilTagProcessor(String name) {
        this(name, new Scalar(0, 0, 255));
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
    protected Collection<VisionObject> processObjects(Mat img, CameraConfig cfg) {
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
        return Arrays.stream(tags)
            .map(obj -> new AprilTag(obj))
            .collect(Collectors.toList());
    }

    @Override
    public void toNetworkTableStateful(NetworkTable table, Ref state) {
        super.toNetworkTableStateful(table, state);
        int size = state.inner.size();
        long[] ids = new long[size];
        double[] transforms = new double[size * 9];
        int i = 0;
        for (VisionObject obj : state.inner) {
            AprilTagDetection tag = ((AprilTag)obj).found;
            ids[i] = tag.getId();
            double[] mat = tag.getHomography();
            for (int j = 0; j < 9; ++j) transforms[i * 9 + j] = mat[j];
        }
        table.putValue("ids", NetworkTableValue.makeIntegerArray(ids));
        table.putValue("transforms", NetworkTableValue.makeDoubleArray(transforms));
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

    public static class Config extends Typed {
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
        public AprilTagProcessor create(String name, Typed cfg) {
            AprilTagProcessor out = new AprilTagProcessor(name);
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
