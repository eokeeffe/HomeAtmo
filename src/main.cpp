/******* INCLUDES *******/
#include <Arduino.h> 

#include <EEPROM.h>
#include <Wire.h>
#include "bsec.h"

#include "WiFi.h"
#include <SPIFFS.h>
#include <HTTPClient.h>
#include <ESPAsyncWebServer.h>
//#include <movingAvg.h>

//#include "html.h"
#include "creds.h"

/******* TIMER/TASK VARIABLES *******/
uint64_t second = 1000;
uint64_t minute = second * 60;
uint64_t hour = minute * 60;
int last_check = 0;
uint64_t last_ifttt = 0;

/******* WI-FI VARIABLES *******/
AsyncWebServer server(80);
HTTPClient http;

/******* SENSOR VARIABLES *******/
const uint8_t bsec_config_iaq[] = {
    #include "bsec_iaq.txt"
};

#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

Bsec sensor;
uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t state_update_counter = 0;

void check_IAQ_sensor_status(void);
void load_state(void);
void update_state(void);

float temperature = 0.0;
float pressure = 0.0;
float humidity = 0.0;
float iaq = 0.0;
float co2Equivalent = 0.0;
const int PM_LENGTH = 12;
float PM[PM_LENGTH];

#define IFTTT_UPDATE_ENABLED 0

bool connection_state = false;

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;

void load_state(void) {
    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
        Serial.println("#### Reading state from EEPROM:");

        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
            bsec_state[i] = EEPROM.read(i + 1);
            Serial.print(bsec_state[i], HEX);
        }
        Serial.println("");

        sensor.setState(bsec_state);
        check_IAQ_sensor_status();
    } else {
        Serial.println("#### Erasing EEPROM");

        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++) {
            EEPROM.write(i, 0);
        }

        EEPROM.commit();
    }
}

void update_state(void) {
    bool update = false;
    if (state_update_counter == 0) {
        if (sensor.iaqAccuracy >= 3) {
            update = true;
            state_update_counter++;
        }
    } else {
        if ((state_update_counter * STATE_SAVE_PERIOD) < millis()) {
            update = true;
            state_update_counter++;
        }
    }

    if (update) {
        sensor.getState(bsec_state);
        check_IAQ_sensor_status();

        Serial.println("#### Writing state to EEPROM:");
        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
            EEPROM.write(i + 1, bsec_state[i]);
            Serial.println(bsec_state[i], HEX);
        }
        Serial.println("");

        EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
        EEPROM.commit();
    }
}

void check_IAQ_sensor_status(void) {
    if (sensor.status != BSEC_OK) {
        if (sensor.status < BSEC_OK) {
            Serial.println("#### BSEC error: " + String(sensor.status));
            for (;;)
                ;
        } else {
            Serial.println("#### BSEC warning: " + String(sensor.status));
        }
    }

    if (sensor.bme680Status != BME680_OK) {
        if (sensor.bme680Status < BME680_OK) {
            Serial.println("#### BME680 error: " + String(sensor.bme680Status));
            for (;;)
                ;
        } else {
            Serial.println("#### BME680 warning: " + String(sensor.bme680Status));
        }
    }
}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }

  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

void setup() {

    for(int i=0;i<PM_LENGTH;i++){
        PM[i] = 0.0;
    }

    Serial.begin(115200);

    Serial.println("#### Setting up sensor.");
    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);

    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    while(!Serial2){}
    Serial.println("Talking with PMS5003");

    Wire.begin();
    sensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);

    check_IAQ_sensor_status();
    sensor.setConfig(bsec_config_iaq);
    check_IAQ_sensor_status();

    load_state();

    bsec_virtual_sensor_t sensor_list[5] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_RAW_PRESSURE
    };

    sensor.updateSubscription(sensor_list, 5, BSEC_SAMPLE_RATE_LP);
    check_IAQ_sensor_status();

    Serial.print("Starting SPIFFS ");
    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    Serial.println("");

    Serial.print("#### Connecting to WiFi");
    WiFi.begin(SSID, PSK);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    connection_state = 1;

    Serial.println("");

    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA ,"bedroom_atmo");
    Serial.println("#### WiFi connected.");
    Serial.print("#### IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("#### Hostname: ");
    Serial.println(WiFi.getHostname());

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    
    server.on("/gauge.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/gauge.min.js", "text/javascript");
    });

    // Sensor status figures
    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(temperature).c_str());
    });
    server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(humidity).c_str());
    });
    server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(pressure / 100.0F).c_str());
    });
    server.on("/iaq", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(iaq).c_str());
    });
    server.on("/co2", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(co2Equivalent).c_str());
    });

    server.on("/pm10_standard", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[0]).c_str());
    });
    server.on("/pm25_standard", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[1]).c_str());
    });
    server.on("/pm100_standard", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[2]).c_str());
    });

    server.on("/pm10_env", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[3]).c_str());
    });
    server.on("/pm25_env", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[4]).c_str());
    });
    server.on("/pm100_env", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[5]).c_str());
    });

    server.on("/particles_03um", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[6]).c_str());
    });
    server.on("/particles_05um", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[7]).c_str());
    });
    server.on("/particles_10um", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[8]).c_str());
    });
    server.on("/particles_25um", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[9]).c_str());
    });
    server.on("/particles_50um", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[10]).c_str());
    });
    server.on("/particles_100um", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(PM[11]).c_str());
    });

    server.begin();
    Serial.println("#### Server Started.");
}

void loop() {

    if ((WiFi.status() != WL_CONNECTED) && (connection_state != 1)) { connection_state = 0; }

    if(!connection_state){
        Serial.println("WIFI Connection dropped");
        Serial.println("Connecting Again");

        
        Serial.print("#### Connecting to WiFi");
        WiFi.begin(SSID, PSK);

        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.print(".");
        }
        connection_state = 1;
    }

    if (millis() - last_check > hour) {
        
        if ((iaq > 100) & IFTTT_UPDATE_ENABLED) {
            Serial.println("BAD QUALITY");

            http.begin(IFTTT);
            http.addHeader("Content-Type", "application/json");
            int httpResponseCode = http.POST("{\"value1\":\"" + String(iaq) + "\"}");

            if (httpResponseCode > 0) {
                String response = http.getString();
                Serial.println(httpResponseCode);
                Serial.println(response);
            } else {
                Serial.print("Error on sending POST: ");
                Serial.println(httpResponseCode);
            }

            http.end();
        }
        last_check = millis();
    }

    if (sensor.run()) {
        temperature = sensor.temperature;
        pressure = sensor.pressure;
        humidity = sensor.humidity;
        co2Equivalent = sensor.co2Equivalent;
        iaq = sensor.iaq;

        

        update_state();
    } else {
        check_IAQ_sensor_status();
    }

    if (readPMSdata(&Serial2)) {
        PM[0]=data.pm10_standard;
        PM[1]=data.pm25_standard;
        PM[2]=data.pm100_standard;
        PM[3]=data.pm10_env;
        PM[4]=data.pm25_env;
        PM[5]=data.pm100_env;
        PM[6]=data.particles_03um;
        PM[7]=data.particles_05um;
        PM[8]=data.particles_10um;
        PM[9]=data.particles_25um;
        PM[10]=data.particles_50um;
        PM[11]=data.particles_100um;
        
        /*
        Serial.println("Concentration Units (standard)");
        Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
        Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
        Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
        Serial.println("---------------------------------------");
        Serial.println("Concentration Units (environmental)");
        Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
        Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
        Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
        Serial.println("---------------------------------------");
        Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
        Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
        Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
        Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
        Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
        Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
        */
    }

    vTaskDelay((second * 5) / portTICK_PERIOD_MS);
}
