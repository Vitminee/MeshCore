#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"

#if defined(ESP32) && defined(WIFI_SSID)
  #include <WiFi.h>
  #ifndef TCP_PORT
    #define TCP_PORT 5000
  #endif
#endif

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[MAX_POST_TEXT_LEN+1];
#define COMMAND_REPLY_MAX_LEN 160
#if defined(ESP32) && defined(WIFI_SSID)
#define WIFI_STATUS_LOG_INTERVAL_MS 10000
static char wifi_command[MAX_POST_TEXT_LEN+1];
static WiFiServer wifi_server(TCP_PORT);
static WiFiClient wifi_client;
static bool wifi_connected_reported = false;
static unsigned long next_wifi_status_log = 0;
#endif

static void handleCommandInput(Stream& input, Print& output, char* cmd_buf, size_t cmd_buf_size, bool echo_input) {
  if (cmd_buf == NULL || cmd_buf_size == 0) return;

  while (input.available()) {
    char c = input.read();
    size_t len = strlen(cmd_buf);

    if (c == '\r' || c == '\n') {
      if (echo_input) output.print(c);
      if (len == 0) continue;

      char reply[COMMAND_REPLY_MAX_LEN];
      the_mesh.handleCommand(0, cmd_buf, reply);  // NOTE: there is no sender_timestamp via serial/WiFi!
      if (reply[0]) {
        output.print("  -> "); output.println(reply);
      }
      cmd_buf[0] = 0;
      continue;
    }

    if (len < cmd_buf_size - 1) {
      cmd_buf[len++] = c;
      cmd_buf[len] = 0;
      if (echo_input) output.print(c);
    } else {
      // command buffer full, process whatever was captured
      char reply[COMMAND_REPLY_MAX_LEN];
      the_mesh.handleCommand(0, cmd_buf, reply);  // NOTE: there is no sender_timestamp via serial/WiFi!
      if (reply[0]) {
        output.print("  -> "); output.println(reply);
      }
      cmd_buf[0] = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_get_rng_seed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Room ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

#if defined(ESP32) && defined(WIFI_SSID)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  Serial.println("Connecting WiFi for room server command channel...");
  wifi_server.begin();
#endif

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif
}

void loop() {
  handleCommandInput(Serial, Serial, command, sizeof(command), true);

#if defined(ESP32) && defined(WIFI_SSID)
  if (WiFi.status() == WL_CONNECTED && !wifi_connected_reported) {
    wifi_connected_reported = true;
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("WiFi command port: ");
    Serial.println(TCP_PORT);
  } else if (WiFi.status() != WL_CONNECTED && millis() >= next_wifi_status_log) {
    next_wifi_status_log = millis() + WIFI_STATUS_LOG_INTERVAL_MS;
    Serial.println("WiFi not connected yet (room server command channel).");
  }

  if (!wifi_client || !wifi_client.connected()) {
    auto next_client = wifi_server.available();
    if (next_client) {
      if (wifi_client) wifi_client.stop();
      wifi_client = next_client;
      wifi_command[0] = 0;
    }
  }
  if (wifi_client && wifi_client.connected()) {
    handleCommandInput(wifi_client, wifi_client, wifi_command, sizeof(wifi_command), false);
  }
#endif

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();
}
