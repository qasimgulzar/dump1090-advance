//
// Created by Muhammad Qasim on 13/07/2025.
//
#include <cjson/cJSON.h>

typedef struct {
    char *msg;
    char *callSign;
    char *icao;
    double speed;
} RawMessage;

typedef struct {
    uint32_t time;
    uint32_t CPR;
    uint32_t surveillanceType;
    uint32_t latCpr;
    uint32_t longCpr;
} LatLongPacket;


char *toRawMessageStr(RawMessage rawMsg) {
    cJSON *json_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(json_obj, "msg", rawMsg.msg);
    cJSON_AddStringToObject(json_obj, "icao", rawMsg.icao);
    cJSON_AddStringToObject(json_obj, "call_sign", rawMsg.callSign);
    cJSON_AddNumberToObject(json_obj, "ground_speed", rawMsg.speed);

    char *json_str = cJSON_PrintUnformatted(json_obj);
    cJSON_Delete(json_obj);
    return json_str;
}
