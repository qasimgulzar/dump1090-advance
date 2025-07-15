//
// Created by Muhammad Qasim on 13/07/2025.
//


#define ADDRESS     "tcp://localhost:1883"
#define CLIENTID    "Dump1090Client-1"
#define TOPIC       "mqttx/aircraft/messages"
#define QOS         1
#define TIMEOUT     10000L

void mqttMessage(char *jsonStr, MQTTClient_message *pubmsg) {
    pubmsg->payload = (void *) jsonStr;
    pubmsg->payloadlen = strlen(jsonStr);
    pubmsg->qos = QOS;
    pubmsg->retained = 0;
}

void mqttPublish(MQTTClient client,char *jsonStr) {
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    mqttMessage(jsonStr, &pubmsg);
    MQTTClient_publishMessage(client, TOPIC, &pubmsg, NULL);
}
