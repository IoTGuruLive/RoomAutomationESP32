/**
 * Device parameters.
 */
#define NUMBER_OF_DEVICES 2

typedef struct {
     String mac;
     
     String deviceId;
     String deviceKey;

     int pwmEnabled;
     int relayEnabled;
} deviceParameters;

deviceParameters deviceParametersMap[NUMBER_OF_DEVICES] = {
    {"807d3af47180", "okB7yhDlvVth6_UQRscR6A", "xxxxxxxxxxxxxxxxxxxxxx", 1, 1}, // Bedroom
    {"30aea40eb638", "mvWLKkpDHL8BVtswK8IR6Q", "xxxxxxxxxxxxxxxxxxxxxx", 1, 1} // ESP32 test
};

/**
 * Node parameters.
 */
#define NUMBER_OF_NODES 2

typedef struct {
     String mac;
     
     String deviceId;
     String nodeId;
     String nodeKey;
} nodeParameters;

nodeParameters nodeParametersMap[NUMBER_OF_NODES] = {
    {"807d3af47180",     "okB7yhDlvVth6_UQRscR6A", "gy8rYTk1GxsVwo8QAMwR5w", "xxxxxxxxxxxxxxxxxxxxxx"}, // Bedroom sensors
    {"30aea40eb638",     "mvWLKkpDHL8BVtswK8IR6Q", "iCvRHsHujSqNj69wK8AR6Q", "xxxxxxxxxxxxxxxxxxxxxx"}, // ESP32 test
};
