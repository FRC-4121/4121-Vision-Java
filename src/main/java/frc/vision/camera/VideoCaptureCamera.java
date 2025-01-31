package frc.vision.camera;

import com.google.gson.*;
import com.google.gson.reflect.TypeToken;
import frc.vision.load.*;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Type;
import java.time.LocalDateTime;
import java.util.Arrays;
import java.util.List;
import org.opencv.core.*;
import org.opencv.videoio.*;

public class VideoCaptureCamera extends CameraBase {
    Mat currentFrame;
    VideoCapture cap;
    int backoff;
    int counter;
    boolean wasOpened;
    int failCount = 0;

    public VideoCaptureCamera(String name, VideoCapture cap, CameraConfig cfg, LocalDateTime date) throws IOException {
        super(name, cfg, date);
        this.currentFrame = new Mat();
        this.cap = cap;
        this.backoff = 1;
        this.wasOpened = cap != null && cap.isOpened();
    }
    public VideoCaptureCamera(String name, Config cfg, LocalDateTime date) throws IOException {
        super(name, cfg, date);
        backoff = 1;
        currentFrame = new Mat();
        if (cfg.index != null) {
            cap = new VideoCapture(cfg.index, Videoio.CAP_V4L2);
        } else if (cfg.path != null) {
            cap = new VideoCapture(cfg.path, Videoio.CAP_V4L2);
        } else if (cfg.port != null) {
            String path = cfg.port.resolve();
            if (path == null) {
                log.write(String.format("Couldn't find a camera on port %s\n", cfg.port));
            } else {
                cap = new VideoCapture(path, Videoio.CAP_V4L2);
            }
        } else {
            log.write("No way to select a camera was given!\n");
        }
        if (!cfg.skipConfig) configureCapture();
        log.flush();
        wasOpened = cap != null && cap.isOpened();
    }

    public VideoCaptureCamera(String name, VideoCapture cap, CameraConfig cfg) throws IOException {
        this(name, cap, cfg, LocalDateTime.now());
    }
    public VideoCaptureCamera(String name, Config cfg) throws IOException {
        this(name, cfg, LocalDateTime.now());
    }

    public void reload() {
        // if (true) return;
        if (cap == null) {
            log.write("Reload requested but capture is null\n");
        } else if (config instanceof Config) {
            log.write("Reloading camera\n");
            Config cfg = (Config)config;
            if (cfg.index != null) {
                cap.open(cfg.index, Videoio.CAP_V4L2);
            } else if (cfg.path != null) {
                cap.open(cfg.path, Videoio.CAP_V4L2);
            } else if (cfg.port != null) {
                try {
                    String path = cfg.port.resolve();
                    if (path == null) {
                        log.write(String.format("Couldn't find a camera on port %s\n", cfg.port));
                    } else {
                        cap = new VideoCapture(path, Videoio.CAP_V4L2);
                    }
                } catch (Exception e) {
                    log.write("Port resolution failed with an exception:\n");
                    e.printStackTrace(log);
                }
            }
            configureCapture();
        } else {
            log.write("Reload requested but we don't know where the camera came from\n");
        }
        log.flush();
    }

    public void configureCapture() {
        if (config instanceof Config) {
            Config cfg = (Config)config;
            if (cap != null) {
                try {
                    if (cfg.fourcc != null) {
                        if (cfg.fourcc.length() == 4) {
                            log.write(String.format("Set fourcc to %s\n", cfg.fourcc));
                            log.flush();
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
                            log.flush();
                        }
                    }
                    if (cfg.width > 0) {
                        log.write(String.format("Set width to %d\n", cfg.width));
                        log.flush();
                        cap.set(Videoio.CAP_PROP_FRAME_WIDTH, cfg.width);
                    }
                    if (cfg.height > 0) {
                        log.write(String.format("Set height to %d\n", cfg.height));
                        log.flush();
                        cap.set(Videoio.CAP_PROP_FRAME_HEIGHT, cfg.height);
                    }
                    if (cfg.fps > 0) {
                        log.write(String.format("Set FPS to %.2f\n", cfg.fps));
                        log.flush();
                        cap.set(Videoio.CAP_PROP_FPS, cfg.fps);
                    }
                    if (cfg.brightness != null) {
                        log.write(String.format("Set brightness to %d\n", cfg.brightness));
                        log.flush();
                        cap.set(Videoio.CAP_PROP_BRIGHTNESS, cfg.brightness);
                    }
                } catch (Exception e) {
                    e.printStackTrace(log);
                    log.flush();
                }
            }
        }
    }

    @Override
    public Mat readFrameRaw() {
        if (cap == null) return null;
        boolean ok = cap.isOpened() && cap.read(currentFrame);
        if (ok) {
            if (!wasOpened) {
                log.write("We got the camera back!\n");
                log.flush();
                wasOpened = true;
            }
            failCount = 0;
            backoff = 1;
            counter = 1;
        } else {
            failCount++;
            if (failCount > 5) {
                if (wasOpened) {
                    log.write("Lost the camera\n");
                    log.flush();
                    wasOpened = false;
                }
                if (counter == 0) {
                    reload();
                    backoff *= 2;
                    counter = backoff;
                } else {
                    counter--;
                }
            }
        }
        log.flush();
        return ok ? currentFrame : null;
    }

    public static class Config extends CameraConfig {
        Integer index;
        String path;
        Port port;

        double fps;
        Double brightness;
        boolean skipConfig;

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
        private static final int[] map4 = {2, 1, 4, 3};
        private static final int[] map5 = {0, 1, 3, 2};
        public int num;
        public PortNum(int num) {
            this.num = num;
        }
        public String resolve() throws IOException {
            if (num >= 4) return null;
            File f = new File(String.format("/sys/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.%d/1-1.%<d:1.0/video4linux", map4[num]));
            if (f.exists()) {
                File[] names = f.listFiles((_dir, name) -> name.startsWith("video") && name.substring(5).matches("\\d+"));
                Arrays.sort(names);
                if (names.length > 0) {
                    return "/dev/" + names[0].getName();
                }
            }
            int port5 = map5[num];
            f = new File(String.format("/sys/devices/platform/axi/1000120000.pcie/1f00%1$d00000.usb/xhci-hcd.%2$d/usb%2$d/%2$d-%3$d/%2$d-%3$d:1.0/video4linux",
                port5 % 2 + 2, port5 % 2, port5 % 2 * 2 + 1, port5 / 2 + 1
            ));
            if (f.exists()) {
                File[] names = f.listFiles((_dir, name) -> name.startsWith("video") && name.substring(5).matches("\\d+"));
                Arrays.sort(names);
                if (names.length > 0) {
                    return "/dev/" + names[0].getName();
                }
            }
            return null;
        }
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
            return new PortList(context.deserialize(json, new TypeToken<List<Port>>() {}.getType()));
        }
    }
}
