package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.junit.jupiter.api.Test;

/** Validates robot configuration without constructing hardware or starting WPILib HAL. */
class RobotConfigurationTest {
  private static final Path DEPLOY_DIR = Path.of("src", "main", "deploy");
  private static final Path SWERVE_DIR = DEPLOY_DIR.resolve(Path.of("swerve", "base"));
  private static final Path PATHPLANNER_DIR = DEPLOY_DIR.resolve("pathplanner");

  @Test
  void deployConfigurationFilesContainValidJson() throws IOException {
    try (Stream<Path> files = Files.walk(DEPLOY_DIR)) {
      List<Path> jsonConfigurations =
          files
              .filter(Files::isRegularFile)
              .filter(RobotConfigurationTest::isJsonConfiguration)
              .toList();

      assertFalse(jsonConfigurations.isEmpty(), "No deploy configuration files were found");
      jsonConfigurations.forEach(
          file ->
              assertDoesNotThrow(
                  () -> parseJson(file), () -> "Invalid JSON configuration: " + file));
    }
  }

  @Test
  void swerveConfigurationReferencesFourValidModules() throws IOException, ParseException {
    JSONObject swerve = parseJson(SWERVE_DIR.resolve("swervedrive.json"));
    JSONArray modules = (JSONArray) swerve.get("modules");

    assertEquals(4, modules.size(), "A swerve drivetrain must define four modules");

    Set<String> moduleNames = new HashSet<>();
    for (Object moduleEntry : modules) {
      String moduleName = (String) moduleEntry;
      assertTrue(moduleNames.add(moduleName), "Duplicate swerve module: " + moduleName);
      assertTrue(
          Files.isRegularFile(SWERVE_DIR.resolve("modules").resolve(moduleName)),
          "Missing swerve module configuration: " + moduleName);
    }
  }

  @Test
  void sparkMotorCanIdsAreValidAndUnique() throws IOException, ParseException {
    JSONObject swerve = parseJson(SWERVE_DIR.resolve("swervedrive.json"));
    JSONArray modules = (JSONArray) swerve.get("modules");
    Set<Long> sparkIds = new HashSet<>();

    for (Object moduleEntry : modules) {
      Path modulePath = SWERVE_DIR.resolve("modules").resolve((String) moduleEntry);
      JSONObject module = parseJson(modulePath);
      addUniqueCanId(sparkIds, deviceId(module, "drive"), modulePath + " drive motor");
      addUniqueCanId(sparkIds, deviceId(module, "angle"), modulePath + " angle motor");
    }

    addUniqueCanId(sparkIds, IndexerConstants.indexerMotor, "indexer motor");
    addUniqueCanId(sparkIds, ShooterConstants.flywheelLeftMotor, "left flywheel motor");
    addUniqueCanId(sparkIds, ShooterConstants.flywheelRightMotor, "right flywheel motor");
  }

  @Test
  void absoluteEncoderCanIdsAreValidAndUnique() throws IOException, ParseException {
    JSONObject swerve = parseJson(SWERVE_DIR.resolve("swervedrive.json"));
    JSONArray modules = (JSONArray) swerve.get("modules");
    Set<Long> encoderIds = new HashSet<>();

    for (Object moduleEntry : modules) {
      Path modulePath = SWERVE_DIR.resolve("modules").resolve((String) moduleEntry);
      JSONObject module = parseJson(modulePath);
      addUniqueCanId(
          encoderIds, deviceId(module, "absoluteEncoder"), modulePath + " absolute encoder");
    }
  }

  @Test
  void pathPlannerAutosReferenceExistingPaths() throws IOException, ParseException {
    Path autosDirectory = PATHPLANNER_DIR.resolve("autos");
    Set<String> referencedPaths = new HashSet<>();

    try (Stream<Path> autos = Files.list(autosDirectory)) {
      List<Path> autoFiles = autos.filter(path -> path.toString().endsWith(".auto")).toList();
      assertFalse(autoFiles.isEmpty(), "No PathPlanner autos were found");

      for (Path autoFile : autoFiles) {
        collectReferencedPaths(parseJson(autoFile), referencedPaths);
      }
    }

    assertFalse(referencedPaths.isEmpty(), "PathPlanner autos do not reference any paths");
    for (String pathName : referencedPaths) {
      assertTrue(
          Files.isRegularFile(PATHPLANNER_DIR.resolve("paths").resolve(pathName + ".path")),
          "PathPlanner auto references missing path: " + pathName);
    }
  }

  private static boolean isJsonConfiguration(Path path) {
    String filename = path.getFileName().toString();
    return filename.endsWith(".json") || filename.endsWith(".path") || filename.endsWith(".auto");
  }

  private static JSONObject parseJson(Path path) throws IOException, ParseException {
    try (Reader reader = Files.newBufferedReader(path)) {
      return (JSONObject) new JSONParser().parse(reader);
    }
  }

  private static long deviceId(JSONObject module, String deviceName) {
    JSONObject device = (JSONObject) module.get(deviceName);
    assertTrue(device != null, "Missing device configuration: " + deviceName);
    Object id = device.get("id");
    assertTrue(id instanceof Number, "Missing CAN ID for device: " + deviceName);
    return ((Number) id).longValue();
  }

  private static void addUniqueCanId(Set<Long> ids, long id, String description) {
    assertTrue(id >= 0 && id <= 62, description + " has invalid CAN ID " + id);
    assertTrue(ids.add(id), description + " duplicates CAN ID " + id);
  }

  private static void collectReferencedPaths(Object node, Set<String> referencedPaths) {
    if (node instanceof JSONObject object) {
      if ("path".equals(object.get("type")) && object.get("data") instanceof JSONObject data) {
        Object pathName = data.get("pathName");
        if (pathName instanceof String name) {
          referencedPaths.add(name);
        }
      }
      for (Object value : object.values()) {
        collectReferencedPaths(value, referencedPaths);
      }
    } else if (node instanceof JSONArray array) {
      for (Object value : array) {
        collectReferencedPaths(value, referencedPaths);
      }
    }
  }
}
