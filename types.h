//
// Created by Muhammad Qasim on 13/07/2025.
//
#include <cjson/cJSON.h>

typedef struct {
    char *msg;
} RawMessage;


char *toRawMessageStr(RawMessage rawMsg) {
    cJSON *json_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(json_obj, "msg", rawMsg.msg);
    char *json_str = cJSON_PrintUnformatted(json_obj);
    cJSON_Delete(json_obj);
    return json_str;
}
