#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient esp32Client;
PubSubClient mqttClient(esp32Client);

const char* ssid = "INFINITUM0D51_2.4";
const char* pass = "XaZ4hU9VCZ";
char* server = "44.206.123.12";
const int mqttPort = 1883;

char* subscribeTopic = "";
char* publishTopic = "esp32.Datos";
const char* mqttUser = "guest";
const char* mqttPassword = "guest";

int var = 0;
char datos[40];
String resultS = "";

void wifiInit() {
    Serial.print("Conectándose a ");
    Serial.println(ssid);

    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("");
    Serial.println("Conectado a WiFi");
    Serial.println("Dirección IP: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Intentando conectarse MQTT...");

        if (mqttClient.connect("arduinoClient", mqttUser, mqttPassword)) {
            Serial.println("Conectado");

            mqttClient.subscribe(subscribeTopic);
        } else {
            Serial.print("Fallo, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" intentar de nuevo en 5 segundos");
            // Esperar 5 segundos antes de intentar de nuevo
            delay(5000);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Mensaje recibido [");
    Serial.print(topic);
    Serial.print("] ");

    char payload_string[length + 1];

    int resultI;

    memcpy(payload_string, payload, length);
    payload_string[length] = '\0';
    resultI = atoi(payload_string);

    var = resultI;
    resultS = "";
    for (int i = 0; i < length; i++) {
        resultS = resultS + (char)payload[i];
    }
    Serial.println();
}

// Agregar las variables relacionadas con el sensor PIR
int pinServo = 4;
int Led = 16;
Servo servo;
int pinLed = 2;
int pinPIR = 12;
bool objetoDetectadoFlag = false;
bool mostrarDeteccion = false;
int contadorObjetos = 0;
unsigned long tiempoDeteccion = 0;
bool servoVueltaOriginal = false; // Indica si el servo ha vuelto a su posición original

// Eliminar la función "readUltrasonicDistance"

void setup() {
    Serial.begin(115200);
    pinMode(pinPIR, INPUT); // Configurar el pin del sensor PIR como entrada
    pinMode(pinLed, OUTPUT);
    pinMode(Led, OUTPUT);
    servo.attach(pinServo);
    wifiInit();
    mqttClient.setServer(server, mqttPort);
    mqttClient.setCallback(callback);
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    // Leer el estado del sensor PIR
    int pirState = digitalRead(pinPIR);

    char payload[50]; // Puedes ajustar el tamaño según los datos que enviarás
    snprintf(payload, sizeof(payload), "Personas" ); // Cambiar el mensaje por "Persona"

    if (pirState == HIGH && !objetoDetectadoFlag) { // Si se detecta movimiento y aún no se ha registrado un objeto detectado
        digitalWrite(pinLed, HIGH);
        objetoDetectadoFlag = true;
        mostrarDeteccion = true;
        contadorObjetos++;
        servo.write(90);
        tiempoDeteccion = millis();
        servoVueltaOriginal = false; // Reiniciar el indicador de retorno a la posición original
    } else if (pirState == LOW) { // Si no se detecta movimiento
        objetoDetectadoFlag = false;
        if (!servoVueltaOriginal && millis() - tiempoDeteccion >= 5000) {
            servo.write(0);
            digitalWrite(pinLed, LOW);
            if (tiempoDeteccion >= 3000) {
                digitalWrite(Led, HIGH);
                delay(1000);
                digitalWrite(Led, LOW);
                delay(10);
            }
            servoVueltaOriginal = true; // Indicar que el servo ha vuelto a la posición original
        }
    }
    if (mostrarDeteccion) {
        Serial.println("A Entrado una Persona");
        Serial.print("||||");
        Serial.print(" Contador: ");
        Serial.println(contadorObjetos);
        mostrarDeteccion = false;
        if (mqttClient.connected()) {
            mqttClient.publish(publishTopic, payload); // Publicar los datos en el tópico MQTT
        }
    }
    delay(10);
}