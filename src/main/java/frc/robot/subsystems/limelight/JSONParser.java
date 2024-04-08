package frc.robot.subsystems.limelight;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

public final class JSONParser {
    private static ObjectMapper mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
   
    public FiducialSnapshot[] read(String json) {        
        try {
            var results = mapper.readValue(json, LimelightResults.class);
            return results.targetingResults.targets_Fiducials;
        } catch (JsonProcessingException e) {
            System.err.println("lljson error: " + e.getMessage());
            return null;
        }
    }

    public static class LimelightResults {
        @JsonProperty("Results")
        public Results targetingResults;

        public LimelightResults() {
            targetingResults = new Results();
        }
    }

    public static class Results {
        @JsonProperty("Fiducial")
        public FiducialSnapshot[] targets_Fiducials;
    }
}