/**
 * Device parameters.
 */
#define NUMBER_OF_DEVICES 1

typedef struct {
     String name;
     String deviceId;
     String deviceKey;

     int pwmEnabled;
     int relayEnabled;
} deviceParameters;

deviceParameters deviceParametersMap[NUMBER_OF_DEVICES] = {
    {"807d3af47180", "0156db30-2bc2-11e9-9af5-8b2a4a431cbf", "xxxxxxxxxxxxxxxxxxxxxx", 1, 1}, // ESP32
};

/**
 * Node parameters.
 */
#define NUMBER_OF_NODES 2

typedef struct {
     String name;
     String nodeId;
     String nodeKey;
     String deviceId;
} nodeParameters;

nodeParameters nodeParametersMap[NUMBER_OF_NODES] = {
    {"28ffb50d5916049a", "8d8faf70-2bc0-11e9-882b-d11ec1ee8d2a", "xxxxxxxxxxxxxxxxxxxxxx", "0156db30-2bc2-11e9-9af5-8b2a4a431cbf"}, // ESP32 DS18B20
    {"807d3af47180",     "8d8faf70-2bc0-11e9-882b-d11ec1ee8d2a", "xxxxxxxxxxxxxxxxxxxxxx", "0156db30-2bc2-11e9-9af5-8b2a4a431cbf"}, // ESP32 MAC
};
