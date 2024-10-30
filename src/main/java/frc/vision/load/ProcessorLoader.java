package frc.vision.load;

import com.google.gson.*;
import com.google.gson.reflect.TypeToken;
import frc.vision.process.VisionProcessor;
import java.lang.reflect.Type;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.Reader;
import java.util.Arrays;
import java.util.List;
import java.util.HashMap;
import java.util.stream.Collectors;

// A camera that can be loaded from the configuration file.
public class ProcessorLoader {
    protected static HashMap<String, ProcessorFactory> types = new HashMap<String, ProcessorFactory>();
    protected static HashMap<String, WrappedConfig> configs = new HashMap<String, WrappedConfig>();
    private static boolean configInitialized = false;

    protected static class WrappedConfig {
        Typed inner;
        public WrappedConfig(Typed inner) {
            this.inner = inner;
        }
    }

    protected static class CustomDeserializer implements JsonDeserializer<WrappedConfig> {
        @Override
        public WrappedConfig deserialize(JsonElement json, Type type, JsonDeserializationContext context) throws JsonParseException {
            JsonObject obj = json.getAsJsonObject();
            String ty = obj.get("type").getAsString();
            ProcessorFactory factory = types.get(ty);
            Typed cfg = context.deserialize(json, factory.configType());
            return new WrappedConfig(cfg);
        }
    }

    protected ProcessorLoader() {}

    // Initialize configs from a file, in JSON.
    public static void initConfig(Reader file) throws JsonSyntaxException, JsonIOException {
        if (configInitialized) {
            System.err.println("Calling ProcessorLoader.initConfig() when already initialized does nothing");
            return;
        }
        Gson gson = new GsonBuilder()
            .registerTypeAdapter(WrappedConfig.class, new CustomDeserializer())
            .create();
        configs = gson.fromJson(file, new TypeToken<HashMap<String, WrappedConfig>>() {}.getType());
        configInitialized = true;
    }
    public static void initConfig() throws JsonSyntaxException, JsonIOException, FileNotFoundException {
        var res = ProcessorLoader.class
            .getClassLoader()
            .getResource("process.json");
        if (res == null) return;
        initConfig(new FileReader(res.getFile()));
    }

    // Register a factory to create cameras.
    public static void registerFactory(ProcessorFactory factory) {
        types.put(factory.typeName(), factory);
    }

    // Load a camera with the given name.
    // TODO: give better exceptions.
    public static VisionProcessor load(String name) {
        WrappedConfig wcfg = configs.get(name);
        Typed cfg = wcfg.inner;
        ProcessorFactory factory = types.get(cfg.type);
        return factory.create(name, cfg);
    }

    public static List<VisionProcessor> loadAll(String... names) {
        return Arrays.stream(names)
            .map(name -> load(name))
            .collect(Collectors.toList());
    }
}
