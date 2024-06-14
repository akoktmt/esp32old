#include "json_utils.h"
#include <string.h>
#include <stdio.h>
#include <cJSON.h>
#define MAX_JSON_LENGTH 256

// Function to prepare JSON data
char *prepare_json_data(float speed, double latitude, double longitude, float heading)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "speed", speed);
    cJSON_AddNumberToObject(root, "lat", latitude);
    cJSON_AddNumberToObject(root, "long", longitude);
    cJSON_AddNumberToObject(root, "heading", heading);
    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    // Truncate JSON string if it exceeds MAX_JSON_LENGTH
    if (strlen(json_string) > MAX_JSON_LENGTH)
    {
        json_string[MAX_JSON_LENGTH - 1] = '\0'; // Null-terminate the string
    }

    // Create a new string with formatted values
    char formatted_json[MAX_JSON_LENGTH];
    snprintf(formatted_json, MAX_JSON_LENGTH, "{\"speed\":%.2f,\"lat\":%.6lf,\"long\":%.6lf,\"heading\":%.2f}", speed, latitude, longitude, heading);

    // Free the original JSON string
    free(json_string);

    // Allocate a new string and copy the formatted JSON data
    char *result = malloc(strlen(formatted_json) + 1);
    strcpy(result, formatted_json);

    return result;
}

char *prepare_sensor_data(float temperature, float humid)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "Temperature", temperature);
    cJSON_AddNumberToObject(root, "Humidity", humid);
    char *json_string = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    // Truncate JSON string if it exceeds MAX_JSON_LENGTH
    if (strlen(json_string) > MAX_JSON_LENGTH)
    {
        json_string[MAX_JSON_LENGTH - 1] = '\0'; // Null-terminate the string
    }

    // Create a new string with formatted values
    char formatted_json[MAX_JSON_LENGTH];
    snprintf(formatted_json, MAX_JSON_LENGTH, "{\"Temperature\":%.2f,\"Humidity\":%.2f}", temperature, humid);

    // Free the original JSON string
    free(json_string);

    // Allocate a new string and copy the formatted JSON data
    char *result = malloc(strlen(formatted_json) + 1);
    strcpy(result, formatted_json);

    return result;
}