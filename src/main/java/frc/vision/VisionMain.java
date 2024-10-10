package frc.vision;

import edu.wpi.first.networktables.*;
import frc.vision.camera.*;
import frc.vision.load.*;
import frc.vision.pipeline.*;
import frc.vision.process.*;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.Arrays;
import java.util.Map;
import java.util.TreeSet;
import java.util.concurrent.*;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;

public class VisionMain {
    private enum CliState {
        NORMAL,
        LOG_DIR,
        ADDRESS,
        NAME,
    };

    protected static final String logNameFormat = "log_%s_%s_%d.txt";

    public static void main(String[] args) throws Exception {
        Map<String, String> env = System.getenv();
        boolean visionDebug = false;
        boolean echoErrors = false;
        TreeSet camNames = new TreeSet();
        {
            String cs = env.getOrDefault("VISION_CAMS", "");
            if (!cs.equals("")) camNames = new TreeSet(Arrays.asList(cs.split(",")));
        }
        File logDir = new File(env.getOrDefault("VISION_LOGS", "logs"));
        String name = env.getOrDefault("NT_IDENTITY", "pi");
        String serverAddress = env.get("NT_SERVER_ADDR");

        CliState state = CliState.NORMAL;
        for (String arg : args) {
            switch (state) {
                case NORMAL:
                    if (arg.charAt(0) == '-') {
                            if (arg.charAt(1) == '-') {
                            String longFlag = arg.substring(2);
                            if (longFlag.equals("vision-debug")) {
                                visionDebug = true;
                            } else if (longFlag.equals("log-dir")) {
                                state = CliState.LOG_DIR;
                            } else if (longFlag.equals("address") || longFlag.equals("server-address")) {
                                state = CliState.ADDRESS;
                            } else if (longFlag.equals("name")) {
                                state = CliState.NAME;
                            } else if (longFlag.equals("echo-errors")) {
                                echoErrors = true;
                            } else {
                                System.err.println(String.format("Unknown long flag \"%s\"", longFlag));
                                System.exit(1);
                            }
                        } else {
                            for (char c : arg.substring(1).toCharArray()) {
                                switch (c) {
                                    case 'v':
                                        visionDebug = true;
                                        break;
                                    case 'l':
                                        if (state != CliState.NORMAL) {
                                            System.err.println("l flag expects the next argument to be the log directory, but another flag already is expecting something");
                                            System.exit(1);
                                        }
                                        state = CliState.LOG_DIR;
                                        break;
                                    case 'a':
                                        if (state != CliState.NORMAL) {
                                            System.err.println("a flag expects the next argument to be the server address, but another flag already is expecting something");
                                            System.exit(1);
                                        }
                                        state = CliState.ADDRESS;
                                        break;
                                    case 'n':
                                        if (state != CliState.NORMAL) {
                                            System.err.println("n flag expects the next argument to be the NT name, but another flag already is expecting something");
                                            System.exit(1);
                                        }
                                        state = CliState.NAME;
                                        break;
                                    case 'e':
                                        echoErrors = true;
                                        break;
                                    default:
                                        System.err.println(String.format("Unknown short flag \"%c\"", c));
                                        System.exit(1);
                                }
                            }
                        }
                    } else {
                        if (camNames.contains(arg)) {
                            System.err.println(String.format("Duplicate camera \"%s\"", arg));
                        }
                        camNames.add(arg);
                    }
                    break;
                case LOG_DIR:
                    logDir = new File(arg);
                    state = CliState.NORMAL;
                    break;
                case ADDRESS:
                    serverAddress = arg;
                    state = CliState.NORMAL;
                    break;
                case NAME:
                    name = arg;
                    state = CliState.NORMAL;
                    break;
            }
        }

        switch (state) {
            case NORMAL: break;
            case LOG_DIR:
                System.err.println("Expected a log directory but no more arguments were passed");
                System.exit(1);
            case ADDRESS:
                System.err.println("Expected the server address but no more arguments were passed");
                System.exit(1);
            case NAME:
                System.err.println("Expected the name to be used but no more arguments were passed");
                System.exit(1);
        }

        if (camNames.isEmpty()) {
            System.err.println("No cameras were specified!");
        }
 
        long pid = ProcessHandle.current().pid();
        System.out.println("PID: " + pid);
        LocalDateTime time = LocalDateTime.now();
        System.out.println("running at " + String.valueOf(time));

        CameraBase.logDir = new File(logDir, "cam");
        CameraBase.echoErrors = echoErrors;
        File runLogs = new File(logDir, "run");
        File runLog = new File(runLogs, String.format(logNameFormat, String.join("_", camNames), CameraBase.logDateFormat.format(time), pid));
        PrintWriter log = new PrintWriter(runLog);
        
        CameraGroup cams = null;

        try {
            log.write(String.format("Running with PID %d at %s\n", pid, time));
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

            CameraLoader.registerFactory(new FrameCamera.Factory());
            CameraLoader.registerFactory(new VideoCaptureCamera.Factory());
            CameraLoader.initConfig();

            ProcessorLoader.registerFactory(new FpsCounter.Factory());
            ProcessorLoader.registerFactory(new AprilTagProcessor.Factory());
            ProcessorLoader.registerFactory(new RectVisionProcessor.Factory());
            ProcessorLoader.initConfig();

            Executor exec = ForkJoinPool.commonPool();

            NetworkTableInstance nt = NetworkTableInstance.getDefault();
            NetworkTable table = null;
            if (serverAddress != null) {
                nt.setServer(serverAddress);
                nt.startClient3(name);
                table = nt.getTable(name);
            }

            VisionLibsGroup procs = new VisionLibsGroup(
                ProcessorLoader.loadAll("fps", "april", "ring2024"),
                table, visionDebug, exec
            );

            {
                String names = procs.getLibs(null)
                    .map(p -> p.getName())
                    .reduce((a, b) -> a + ", " + b)
                    .orElse("<none>");
                log.write(String.format("Loaded processors: %s\n", names));
            }
            

            ImShower imgs = new ImShower();
            if (visionDebug) {
               procs.setPostProcess(imgs);
            }

            cams = CameraGroup.of(camNames);
            cams.setCallback(procs);

            {
                String names = cams.getCams()
                    .map(p -> p.getCamera().getName())
                    .reduce((a, b) -> a + ", " + b)
                    .orElse("<none>");
                log.write(String.format("Loaded cameras: %s\n", names));
            }
            
            log.flush();

            cams.start();


            System.out.println("Running successfully :3");
            
            while (true) {
                if (visionDebug) {
                    imgs.run();
                    if (imgs.canWaitKey()) {
                        int res = HighGui.waitKey(10);
                        if (res == 27) {
                            log.write("ESC key pressed\n");
                            break;
                        }
                    }
                }
            }

            cams.cancel();

            if (visionDebug) HighGui.waitKey(1); // appease opencv
        } catch (Exception e) {
            e.printStackTrace(log);
            if (echoErrors) e.printStackTrace();
        } finally {
            if (cams != null) cams.flushLogs();
            log.write(String.format("Run ended at %s\n", String.valueOf(LocalDateTime.now())));
            log.flush();
        }
    }
}
