package frc.vision.camera;

import com.google.gson.*;
import frc.vision.load.*;
import java.io.IOException;
import java.lang.reflect.Type;
import java.time.LocalDateTime;
import java.util.List;
import org.opencv.core.*;
import org.opencv.videoio.*;

public class VideoCaptureCamera extends CameraBase {
    Mat currentFrame;
    VideoCapture cap;
    int backoff;
    int counter;

    public VideoCaptureCamera(String name, VideoCapture cap, CameraConfig cfg, LocalDateTime date) throws IOException {
        super(name, cfg, date);
        this.currentFrame = new Mat();
        this.cap = cap;
        this.backoff = 1;
    }
    public VideoCaptureCamera(String name, Config cfg, LocalDateTime date) throws IOException {
        super(name, cfg, date);
        backoff = 1;
        currentFrame = new Mat();
        if (cfg.index != null) {
            cap = new VideoCapture(cfg.index);
        } else if (cfg.path != null) {
            cap = new VideoCapture(cfg.path);
        } else if (cfg.port != null) {
            String path = cfg.port.resolve();
            if (path == null) {
                log.write(String.format("Couldn't find a camera on port %s\n", cfg.port));
            } else {
                cap = new VideoCapture(path);
            }
        } else {
            log.write("No way to select a camera was given!\n");
        }
        if (cap != null) {
            try {
                if (cfg.width != 0) {
                    cap.set(Videoio.CAP_PROP_FRAME_WIDTH, cfg.width);
                }
                if (cfg.height != 0) {
                    cap.set(Videoio.CAP_PROP_FRAME_WIDTH, cfg.width);
                }
                if (cfg.fps != 0) {
                    cap.set(Videoio.CAP_PROP_FPS, cfg.fps);
                }
                if (cfg.brightness != null) {
                    cap.set(Videoio.CAP_PROP_BRIGHTNESS, cfg.brightness);
                }
                if (cfg.fourcc != null) {
                    if (cfg.fourcc.length() == 4) {
                        cap.set(Videoio.CAP_PROP_FOURCC,
                            VideoWriter.fourcc(
                                cfg.fourcc.charAt(0),
                                cfg.fourcc.charAt(1),
                                cfg.fourcc.charAt(2),
                                cfg.fourcc.charAt(3)
                            )
                        );
                    } else {
                        log.write(String.format("Invalid fourcc \"%s\"\n", cfg.fourcc));
                    }
                }
            } catch (Exception e) {
                e.printStackTrace(log);
            }
        }
        log.flush();
    }

    public VideoCaptureCamera(String name, VideoCapture cap, CameraConfig cfg) throws IOException {
        this(name, cap, cfg, LocalDateTime.now());
    }
    public VideoCaptureCamera(String name, Config cfg) throws IOException {
        this(name, cfg, LocalDateTime.now());
    }

    public void reload() {
        if (cap == null) {
            log.write("Reload requested but capture is null");
        } else if (config instanceof Config) {
            log.write("Reloading camera");
            Config cfg = (Config)config;
            if (cfg.index != null) {
                cap.open(cfg.index);
            } else if (cfg.path != null) {
                cap.open(cfg.path);
            } else if (cfg.port != null) {
                try {
                    String path = cfg.port.resolve();
                    if (path == null) {
                        log.write(String.format("Couldn't find a camera on port %s\n", cfg.port));
                    } else {
                        cap = new VideoCapture(path);
                    }
                } catch (Exception e) {
                    log.write("Port resolution failed with an exception:\n");
                    e.printStackTrace(log);
                }
            }
        } else {
            log.write("Reload requested but we don't know where the camera came from\n");
        }
    }
    
    @Override
    public Mat readFrameRaw() {
        if (cap == null) return null;
        if (cap.isOpened()) {
            backoff = 1;
            counter = 1;
        } else {
            if (counter == 0) {
                reload();
                backoff *= 2;
                counter = backoff;
            } else {
                counter--;
            }
        }
        boolean ok = cap.read(currentFrame);
        return ok ? currentFrame : null;
    }

    public static class Config extends CameraConfig {
        Integer index;
        String path;
        Port port;

        double fps;
        Double brightness;

        String fourcc;
    }

    public static class Factory extends CameraFactory {
        @Override
        public Class<Config> configType() {
            return Config.class;
        }

        @Override
        public String typeName() {
            return "cv";
        }

        @Override
        public void modifyBuilder(GsonBuilder builder) {
            builder.registerTypeAdapter(Port.class, new PortDeserializer());
        }

        @Override
        public VideoCaptureCamera create(String name, CameraConfig cfg, LocalDateTime date) throws IOException {
            return new VideoCaptureCamera(name, (Config)cfg, date);
        }
    }

    // A port to be resolved
    public static interface Port {
        String resolve() throws IOException;
    }

    // A numeric identifier corresponding to a physical port
    public static class PortNum implements Port {
        public int num;
        public PortNum(int num) {
            this.num = num;
        }
        public String resolve() throws IOException { return null; }
        public String toString() {
            return String.valueOf(num);
        }
    }
    // A hardware name for a physical port
    public static class PortName implements Port {
        public String name;
        public PortName(String name) {
            this.name = name;
        }
        public String resolve() throws IOException { return null; }
        public String toString() {
            return String.format("\"%s\"", name);
        }
    }
    // A list of port identifiers to try
    public static class PortList implements Port {
        public List<Port> ports;
        public PortList(List<Port> ports) {
            this.ports = ports;
        }
        public String resolve() throws IOException {
            IOException e = null;
            for (Port p : ports) {
                try {
                    String res = p.resolve();
                    if (res != null) return res;
                } catch (IOException ex) {
                    e = ex;
                }
            }
            if (e != null) throw e;
            return null;
        }
        public String toString() {
            return "[" + ports.stream().map(p -> p.toString()).reduce((a, b) -> a + "," + b) + "]";
        }
    }

    protected static class PortDeserializer implements JsonDeserializer<Port> {
        @Override
        public Port deserialize(JsonElement json, Type type, JsonDeserializationContext context) throws JsonParseException {
            if (json instanceof JsonPrimitive) {
                JsonPrimitive prim = (JsonPrimitive)json;
                if (prim.isNumber()) {
                    return new PortNum(prim.getAsInt());
                }
                if (prim.isString()) {
                    return new PortName(prim.getAsString());
                }
            }
            return new PortList(context.deserialize(json, List.class));
        }
    }
}
