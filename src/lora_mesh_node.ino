#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "mbedtls/gcm.h"

//==================================================
// --- USER CONFIGURATION ---
//==================================================

// CHANGE THIS FOR EACH BOARD: 1, 2, or 3
#define NODE_ID 3 

// NETWORK SETTINGS
#define MAX_NODES 5             
#define HELLO_INTERVAL 2000     // Send "Hello" every 2 seconds
#define ROUTE_TIMEOUT 15000     // Remove routes not seen for 15 seconds
#define MAX_HOPS 3              // Max hop count (prevents count-to-infinity)
#define DISPLAY_TIMEOUT 3000    // Return to idle display after 3 seconds

// BUTTON SETTINGS
#define BUTTON_UP_PIN 13        // Keep original node-select flow on this pin
#define BUTTON_DOWN_PIN 26
#define BUTTON_LEFT_PIN 25
#define BUTTON_RIGHT_PIN 27
#define DEBOUNCE_DELAY 50       // Debounce time in ms
#define LONG_PRESS_TIME 500     // Long press threshold in ms
#define MAX_COMPOSE_LEN 24      // Max typed message length

// DUAL-CORE QUEUE SETTINGS
#define MSG_QUEUE_SIZE 10       // Max queued received messages
#define RELAY_QUEUE_SIZE 10     // Max queued relay requests
#define MAX_PAYLOAD_LEN 128     // Max message payload length

// ENCRYPTION KEY (Must match on all boards)
unsigned char aes_key[16] = {
  0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
  0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};

//==================================================
// --- PINS & HARDWARE ---
//==================================================
#define RST     14
#define DIO0    2
#define LORA_FREQUENCY 433E6

// OLED DISPLAY (128x64)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

//==================================================
// --- ROUTING TABLE ---
//==================================================
struct RoutingEntry {
  int nodeId;       
  int nextHop;      
  int distance;     
  unsigned long lastSeen; 
};

RoutingEntry routingTable[MAX_NODES];
int routingTableSize = 0;

//==================================================
// --- QUEUED MESSAGE STRUCTS ---
//==================================================
struct RxDataMessage {
  int src;
  char payload[MAX_PAYLOAD_LEN];
};

struct RelayRequest {
  int origSrc;
  int dst;
  char payload[MAX_PAYLOAD_LEN];
};

//==================================================
// --- GLOBAL VARIABLES ---
//==================================================
unsigned long lastHelloTime = 0;
unsigned long lastDisplayUpdate = 0;

// Button state machine
enum ButtonIndex { BTN_UP = 0, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_COUNT };
const int BUTTON_PINS[BTN_COUNT] = { BUTTON_UP_PIN, BUTTON_DOWN_PIN, BUTTON_LEFT_PIN, BUTTON_RIGHT_PIN };

int selectedTargetNode = 0;       // 0 = none, 1, 2, 3 = target nodes
int stableButtonState[BTN_COUNT] = { HIGH, HIGH, HIGH, HIGH };
int lastReadingState[BTN_COUNT] = { HIGH, HIGH, HIGH, HIGH };
unsigned long buttonLastDebounce[BTN_COUNT] = { 0, 0, 0, 0 };
bool buttonPressed[BTN_COUNT] = { false, false, false, false };
bool longPressHandled[BTN_COUNT] = { false, false, false, false };
unsigned long buttonPressStart[BTN_COUNT] = { 0, 0, 0, 0 };

// On-screen keyboard state
const int TEXT_AREA_HEIGHT = 18;
const int KEY_ROWS = 4;
const char* keyLayout[KEY_ROWS][11] = {
  {"Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", NULL},
  {"A", "S", "D", "F", "G", "H", "J", "K", "L", NULL},
  {"Z", "X", "C", "V", "B", "N", "M", ",", ".", NULL},
  {"Spc", "<-", "SEND", "CNL", NULL}
};
int cursorRow = 0;
int cursorCol = 0;
char composedMessage[MAX_COMPOSE_LEN + 1] = "";
int composedLen = 0;
unsigned long lastKeyboardBlinkRefresh = 0;

// Display state
enum DisplayState { IDLE, SELECTING, COMPOSING, SENDING, RECEIVED, RELAYING };
DisplayState currentDisplayState = IDLE;
unsigned long displayStateTime = 0;
String lastReceivedFrom = "";
String lastReceivedMsg = "";

// FreeRTOS handles (dual-core)
QueueHandle_t rxDisplayQueue;     // Messages queued for display
QueueHandle_t relayQueue;         // Messages queued for relay
SemaphoreHandle_t loraMutex;      // Protects LoRa SPI access
SemaphoreHandle_t routeMutex;     // Protects routing table access
TaskHandle_t receiverTaskHandle;  // Receiver task on Core 0
volatile int pendingMsgCount = 0; // Unread message counter

// Snapshot of routing table for thread-safe display
int displayRouteCount = 0;
int displayRouteNodes[MAX_NODES];
int displayRouteHops[MAX_NODES];
int displayRouteDist[MAX_NODES];

//==================================================
// --- TARGET NODE SELECTION ---
//==================================================
// Returns available target nodes (excluding self)
int getNextTargetNode(int current) {
  int targets[2];
  int count = 0;
  
  for (int i = 1; i <= 3; i++) {
    if (i != NODE_ID) {
      targets[count++] = i;
    }
  }
  
  if (current == 0) return targets[0];
  
  for (int i = 0; i < count; i++) {
    if (targets[i] == current) {
      return targets[(i + 1) % count];
    }
  }
  return targets[0];
}

//==================================================
// --- DISPLAY HELPERS ---
//==================================================
void drawHeader() {
  u8g2.setFont(u8g2_font_6x10_tf);
  
  String header = "Node " + String(NODE_ID);
  u8g2.drawStr(0, 10, header.c_str());
  
  // Show pending message badge in center
  if (pendingMsgCount > 0) {
    String badge = "[" + String(pendingMsgCount) + " msg]";
    int bw = u8g2.getStrWidth(badge.c_str());
    u8g2.drawStr(64 - bw / 2, 10, badge.c_str());
  }
  
  // Draw neighbor count on right (from snapshot)
  String neighbors = "N:" + String(displayRouteCount);
  int width = u8g2.getStrWidth(neighbors.c_str());
  u8g2.drawStr(128 - width, 10, neighbors.c_str());
  
  u8g2.drawHLine(0, 13, 128);
}

void displayIdle() {
  u8g2.clearBuffer();
  drawHeader();
  
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 28, "UP: Select node");
  u8g2.drawStr(0, 40, "Hold UP: keyboard");
  
  // Show routing info (from thread-safe snapshot)
  u8g2.drawHLine(0, 48, 128);
  u8g2.setFont(u8g2_font_5x7_tf);
  
  if (displayRouteCount == 0) {
    u8g2.drawStr(0, 58, "No routes discovered");
  } else {
    String routeInfo = "";
    for (int i = 0; i < displayRouteCount && i < 3; i++) {
      if (i > 0) routeInfo += " ";
      routeInfo += String(displayRouteNodes[i]) + "v" + String(displayRouteHops[i]) + "d" + String(displayRouteDist[i]);
    }
    u8g2.drawStr(0, 58, routeInfo.c_str());
  }
  
  u8g2.sendBuffer();
}

int getNumKeysInRow(int row) {
  int numKeys = 0;
  while (keyLayout[row][numKeys] != NULL) {
    numKeys++;
  }
  return numKeys;
}

void resetKeyboardComposer() {
  cursorRow = 0;
  cursorCol = 0;
  composedLen = 0;
  composedMessage[0] = '\0';
}

void displayKeyboard() {
  u8g2.clearBuffer();

  // Keyboard-only UI (no header line)
  const int textAreaX = 2;
  const int textAreaY = 2;
  const int textAreaW = 124;
  const int textAreaH = 14;
  const int keyboardTop = textAreaY + textAreaH + 4;
  const int outerMarginX = 3;
  const int outerMarginBottom = 2;
  const int rowGap = 2;

  u8g2.setFontPosTop();
  u8g2.drawFrame(textAreaX, textAreaY, textAreaW, textAreaH);
  u8g2.setFont(u8g2_font_5x7_tf);

  char msgBuffer[MAX_COMPOSE_LEN + 2];
  strncpy(msgBuffer, composedMessage, sizeof(msgBuffer) - 1);
  msgBuffer[sizeof(msgBuffer) - 1] = '\0';

  // Blinking cursor inside text area
  if ((millis() / 400) % 2 == 0 && composedLen < MAX_COMPOSE_LEN) {
    msgBuffer[composedLen] = '_';
    msgBuffer[composedLen + 1] = '\0';
  }
  u8g2.drawStr(textAreaX + 3, textAreaY + 3, msgBuffer);

  int availableKeyboardH = 64 - keyboardTop - outerMarginBottom;
  int keyHeight = (availableKeyboardH - (KEY_ROWS - 1) * rowGap) / KEY_ROWS;
  int availableKeyboardW = 128 - (2 * outerMarginX);

  for (int row = 0; row < KEY_ROWS; row++) {
    int numKeys = getNumKeysInRow(row);
    int keyWidth = availableKeyboardW / numKeys;
    int rowY = keyboardTop + row * (keyHeight + rowGap);

    // Draw one row frame and internal separators (grid style)
    u8g2.drawFrame(outerMarginX, rowY, availableKeyboardW, keyHeight);
    for (int col = 1; col < numKeys; col++) {
      int separatorX = outerMarginX + (col * keyWidth);
      u8g2.drawVLine(separatorX, rowY, keyHeight);
    }

    for (int col = 0; col < numKeys; col++) {
      int cellX = outerMarginX + (col * keyWidth);
      int cellY = rowY;
      int nextCellX = (col == numKeys - 1) ? (outerMarginX + availableKeyboardW) : (outerMarginX + ((col + 1) * keyWidth));
      int cellW = nextCellX - cellX;
      const char* keyLabel = keyLayout[row][col];

      // Selection highlight in current grid cell
      if (row == cursorRow && col == cursorCol) {
        if (cellW > 2 && keyHeight > 2) {
          u8g2.drawBox(cellX + 1, cellY + 1, cellW - 2, keyHeight - 2);
        }
        u8g2.setDrawColor(0);
      }

      u8g2.setFont(u8g2_font_5x7_tf);
      int labelWidth = u8g2.getStrWidth(keyLabel);
      int textX = cellX + (cellW - labelWidth) / 2;
      int textY = cellY + (keyHeight - 7) / 2;
      u8g2.drawStr(textX, textY, keyLabel);
      u8g2.setDrawColor(1);
    }
  }

  u8g2.setFontPosBaseline();
  u8g2.sendBuffer();
}

void moveKeyboardCursor(int rowDelta, int colDelta) {
  if (rowDelta != 0) {
    cursorRow += rowDelta;
    if (cursorRow < 0) cursorRow = KEY_ROWS - 1;
    if (cursorRow >= KEY_ROWS) cursorRow = 0;
  }

  int rowSize = getNumKeysInRow(cursorRow);
  if (colDelta != 0) {
    cursorCol += colDelta;
    if (cursorCol < 0) cursorCol = rowSize - 1;
    if (cursorCol >= rowSize) cursorCol = 0;
  }

  if (cursorCol >= rowSize) cursorCol = rowSize - 1;
}

void applyKeyboardSelection() {
  const char* selectedKey = keyLayout[cursorRow][cursorCol];

  if (strcmp(selectedKey, "<-") == 0) {
    if (composedLen > 0) {
      composedLen--;
      composedMessage[composedLen] = '\0';
    }
    displayKeyboard();
    return;
  }

  if (strcmp(selectedKey, "Spc") == 0) {
    if (composedLen < MAX_COMPOSE_LEN) {
      composedMessage[composedLen++] = ' ';
      composedMessage[composedLen] = '\0';
    }
    displayKeyboard();
    return;
  }

  if (strcmp(selectedKey, "CNL") == 0) {
    resetKeyboardComposer();
    currentDisplayState = IDLE;
    selectedTargetNode = 0;
    displayIdle();
    return;
  }

  if (strcmp(selectedKey, "SEND") == 0) {
    if (composedLen > 0 && selectedTargetNode != 0) {
      sendData(selectedTargetNode, String(composedMessage));
      resetKeyboardComposer();
      selectedTargetNode = 0;
    }
    return;
  }

  if (composedLen < MAX_COMPOSE_LEN) {
    composedMessage[composedLen++] = selectedKey[0];
    composedMessage[composedLen] = '\0';
  }

  displayKeyboard();
}

void displaySelecting() {
  u8g2.clearBuffer();
  drawHeader();
  
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(0, 28, "SELECT TARGET:");
  
  // Draw node indicator boxes with numbers below
  int boxY = 38;
  int boxSize = 14;
  int spacing = 36;
  int startX = 64 - spacing; // Center 3 boxes
  
  u8g2.setFont(u8g2_font_6x10_tf);
  
  for (int i = 1; i <= 3; i++) {
    int x = startX + (i - 1) * spacing;
    int boxCenterX = x - (boxSize / 2);
    
    if (i == NODE_ID) {
      // Self - filled box (you can't select yourself)
      u8g2.drawBox(boxCenterX, boxY, boxSize, boxSize);
    } else if (i == selectedTargetNode) {
      // Selected target - filled with border
      u8g2.drawBox(boxCenterX, boxY, boxSize, boxSize);
      u8g2.drawFrame(boxCenterX - 2, boxY - 2, boxSize + 4, boxSize + 4);
    } else {
      // Other nodes - empty box
      u8g2.drawFrame(boxCenterX, boxY, boxSize, boxSize);
    }
    
    // Draw node number below the box
    String numStr = String(i);
    int numWidth = u8g2.getStrWidth(numStr.c_str());
    u8g2.drawStr(x - (numWidth / 2), boxY + boxSize + 10, numStr.c_str());
  }
  
  u8g2.sendBuffer();
}

void displaySending(int targetNode) {
  u8g2.clearBuffer();
  drawHeader();
  
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(0, 30, "SENDING TO:");
  
  // Target node
  u8g2.setFont(u8g2_font_logisoso18_tn);
  String targetStr = "Node " + String(targetNode);
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(20, 48, targetStr.c_str());
  
  // Show routing
  int nextHop = getNextHop(targetNode);
  u8g2.setFont(u8g2_font_5x7_tf);
  String via = "via: ";
  if (nextHop == targetNode) {
    via += "Direct";
  } else if (nextHop != -1) {
    via += "Node " + String(nextHop);
  } else {
    via += "Unknown";
  }
  u8g2.drawStr(0, 62, via.c_str());
  
  u8g2.sendBuffer();
}

void displayReceived(String from, String msg) {
  u8g2.clearBuffer();
  drawHeader();
  
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(0, 28, "MSG RECEIVED!");
  
  u8g2.setFont(u8g2_font_6x10_tf);
  String fromStr = "From: Node " + from;
  u8g2.drawStr(0, 42, fromStr.c_str());
  
  // Truncate message if too long
  u8g2.setFont(u8g2_font_5x7_tf);
  String displayMsg = msg;
  if (displayMsg.length() > 24) {
    displayMsg = displayMsg.substring(0, 21) + "...";
  }
  u8g2.drawStr(0, 56, displayMsg.c_str());
  
  u8g2.sendBuffer();
}

void displayRelaying(int src, int dst, int nextHop) {
  u8g2.clearBuffer();
  drawHeader();
  
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(0, 28, "RELAYING...");
  
  u8g2.setFont(u8g2_font_6x10_tf);
  String route = String(src) + " -> " + String(NODE_ID) + " -> " + String(nextHop);
  u8g2.drawStr(0, 44, route.c_str());
  
  String dest = "Final: Node " + String(dst);
  u8g2.drawStr(0, 58, dest.c_str());
  
  u8g2.sendBuffer();
}

//==================================================
// --- ROUTING LOGIC ---
//==================================================
void updateRoute(int targetNode, int viaNode, int dist) {
  if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  
  for (int i = 0; i < routingTableSize; i++) {
    if (routingTable[i].nodeId == targetNode) {
      if (dist < routingTable[i].distance) {
        // Shorter path found - switch to it and refresh
        routingTable[i].nextHop = viaNode;
        routingTable[i].distance = dist;
        routingTable[i].lastSeen = millis();
        Serial.printf("Route Updated: To %d via %d (Dist: %d) [shorter]\n", targetNode, viaNode, dist);
      } else if (routingTable[i].nextHop == viaNode) {
        // Same next hop we're using - refresh lastSeen
        routingTable[i].lastSeen = millis();
        if (dist != routingTable[i].distance) {
          routingTable[i].distance = dist;
          Serial.printf("Route Updated: To %d via %d (Dist: %d) [topo change]\n", targetNode, viaNode, dist);
        }
      }
      // Otherwise: worse/equal distance from DIFFERENT node - DON'T refresh lastSeen
      // This ensures routes expire when the actual next-hop node goes down
      
      xSemaphoreGive(routeMutex);
      return;
    }
  }

  if (routingTableSize < MAX_NODES) {
    routingTable[routingTableSize].nodeId = targetNode;
    routingTable[routingTableSize].nextHop = viaNode;
    routingTable[routingTableSize].distance = dist;
    routingTable[routingTableSize].lastSeen = millis();
    routingTableSize++;
    Serial.printf("Route Added: To %d via %d (Dist: %d)\n", targetNode, viaNode, dist);
  }
  xSemaphoreGive(routeMutex);
}

int getNextHop(int destination) {
  int result = -1;
  if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    for (int i = 0; i < routingTableSize; i++) {
      if (routingTable[i].nodeId == destination) {
        result = routingTable[i].nextHop;
        break;
      }
    }
    xSemaphoreGive(routeMutex);
  }
  return result;
}

//==================================================
// --- ROUTE EXPIRATION ---
//==================================================
void expireOldRoutes() {
  if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  unsigned long now = millis();
  
  for (int i = 0; i < routingTableSize; ) {
    if (now - routingTable[i].lastSeen > ROUTE_TIMEOUT) {
      Serial.printf("Route Expired: Node %d (not seen for %lu ms)\n", 
                    routingTable[i].nodeId, now - routingTable[i].lastSeen);
      
      for (int j = i; j < routingTableSize - 1; j++) {
        routingTable[j] = routingTable[j + 1];
      }
      routingTableSize--;
    } else {
      i++;
    }
  }
  xSemaphoreGive(routeMutex);
}

//==================================================
// --- IMPLICIT ROUTE WITHDRAWAL ---
//==================================================
// When we receive a HELLO from node X, check if X stopped
// advertising any route we depend on through X. If so, remove it.
void withdrawMissingRoutes(int viaNode, int* advertisedNodes, int advertisedCount) {
  if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  
  for (int i = 0; i < routingTableSize; ) {
    // Only check routes that go THROUGH viaNode (not the direct route TO viaNode)
    if (routingTable[i].nextHop == viaNode && routingTable[i].nodeId != viaNode) {
      bool stillAdvertised = false;
      for (int j = 0; j < advertisedCount; j++) {
        if (advertisedNodes[j] == routingTable[i].nodeId) {
          stillAdvertised = true;
          break;
        }
      }
      if (!stillAdvertised) {
        Serial.printf("Route Withdrawn: Node %d via %d (no longer advertised)\n",
                      routingTable[i].nodeId, viaNode);
        for (int j = i; j < routingTableSize - 1; j++) {
          routingTable[j] = routingTable[j + 1];
        }
        routingTableSize--;
        continue; // Don't increment i
      }
    }
    i++;
  }
  
  xSemaphoreGive(routeMutex);
}

//==================================================
// --- ENCRYPTION & SENDING ---
//==================================================
void sendRawPacket(String type, int src, int dst, int nextHop, String payload) {
  String packet = type + "|" + String(src) + "|" + String(dst) + "|" + String(nextHop) + "|" + payload;

  unsigned char plaintext[256];
  unsigned char iv[12];  // GCM typically uses 12-byte nonce
  unsigned char ciphertext[256];
  unsigned char tag[16]; // Authentication tag
  mbedtls_gcm_context gcm_ctx;

  strcpy((char*)plaintext, packet.c_str());
  size_t plaintext_len = strlen((char*)plaintext);
  
  // GCM doesn't need padding
  
  // Generate random IV/nonce
  for (int i = 0; i < 12; i++) iv[i] = (unsigned char)esp_random();

  mbedtls_gcm_init(&gcm_ctx);
  mbedtls_gcm_setkey(&gcm_ctx, MBEDTLS_CIPHER_ID_AES, aes_key, 128);
  
  // Encrypt and generate authentication tag
  mbedtls_gcm_crypt_and_tag(&gcm_ctx, MBEDTLS_GCM_ENCRYPT, plaintext_len,
                            iv, 12, NULL, 0,  // No additional authenticated data
                            plaintext, ciphertext, 16, tag);
  
  mbedtls_gcm_free(&gcm_ctx);

  // Acquire LoRa mutex for SPI-safe transmission
  if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    LoRa.beginPacket();
    LoRa.write(iv, 12);
    LoRa.write(ciphertext, plaintext_len);
    LoRa.write(tag, 16);
    LoRa.endPacket();
    LoRa.receive(); // Return to continuous RX immediately after send
    xSemaphoreGive(loraMutex);
  }
  
  if (type == "D") Serial.println("TX Data: " + packet);
}

void sendHello() {
  String routes = "";
  if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    for (int i = 0; i < routingTableSize; i++) {
      // Split-horizon: only advertise direct neighbors (distance 1)
      // This prevents count-to-infinity where nodes echo routes back
      if (routingTable[i].distance == 1) {
        routes += String(routingTable[i].nodeId) + ":" + String(routingTable[i].distance) + ",";
      }
    }
    xSemaphoreGive(routeMutex);
  }
  sendRawPacket("H", NODE_ID, 0, 0, routes); 
}

void sendData(int destNode, String msg) {
  int nextHop = getNextHop(destNode);
  if (nextHop == -1) nextHop = destNode;

  displaySending(destNode);
  currentDisplayState = SENDING;
  displayStateTime = millis();
  
  sendRawPacket("D", NODE_ID, destNode, nextHop, msg);
  Serial.printf("Sent to Node %d via %d: %s\n", destNode, nextHop, msg.c_str());
}

//==================================================
// --- PACKET PROCESSING (Called from Core 0) ---
//==================================================
void processPacket(uint8_t* rawData, int rawLen) {
  if (rawLen < 32) return;

  unsigned char iv[12];
  memcpy(iv, rawData, 12);
  
  size_t ciphertext_len = rawLen - 12 - 16;
  unsigned char ciphertext[ciphertext_len];
  memcpy(ciphertext, rawData + 12, ciphertext_len);
  
  unsigned char tag[16];
  memcpy(tag, rawData + 12 + ciphertext_len, 16);

  unsigned char decrypted_buffer[ciphertext_len + 1];
  mbedtls_gcm_context gcm_ctx;
  
  mbedtls_gcm_init(&gcm_ctx);
  mbedtls_gcm_setkey(&gcm_ctx, MBEDTLS_CIPHER_ID_AES, aes_key, 128);
  
  int ret = mbedtls_gcm_auth_decrypt(&gcm_ctx, ciphertext_len,
                                      iv, 12, NULL, 0,
                                      tag, 16, ciphertext, decrypted_buffer);
  
  mbedtls_gcm_free(&gcm_ctx);
  
  if (ret != 0) {
    Serial.println("Auth failed! Packet rejected.");
    return;
  }
  
  decrypted_buffer[ciphertext_len] = '\0';

  String receivedMsg = String((char*)decrypted_buffer);

  int split1 = receivedMsg.indexOf('|');
  int split2 = receivedMsg.indexOf('|', split1 + 1);
  int split3 = receivedMsg.indexOf('|', split2 + 1);
  int split4 = receivedMsg.indexOf('|', split3 + 1);

  if (split1 == -1 || split4 == -1) return;

  String type = receivedMsg.substring(0, split1);
  int src = receivedMsg.substring(split1 + 1, split2).toInt();
  int dst = receivedMsg.substring(split2 + 1, split3).toInt();
  int next = receivedMsg.substring(split3 + 1, split4).toInt();
  String payload = receivedMsg.substring(split4 + 1);

  // Ignore our own packets (LoRa can hear its own transmissions)
  if (src == NODE_ID) return;

  // No artificial barrier - relies on actual physical distance
  updateRoute(src, src, 1); 

  // --- PROCESS HELLO ---
  if (type == "H") {
    int advertisedNodes[MAX_NODES];
    int advertisedCount = 0;
    
    int idx = 0;
    while (idx < (int)payload.length()) {
      int comma = payload.indexOf(',', idx);
      if (comma == -1) comma = payload.length();
      String pair = payload.substring(idx, comma);
      int colon = pair.indexOf(':');
      if (colon != -1) {
        int rId = pair.substring(0, colon).toInt();
        int rDist = pair.substring(colon + 1).toInt();
        if (rId != NODE_ID && rDist + 1 <= MAX_HOPS) {
          updateRoute(rId, src, rDist + 1);
          if (advertisedCount < MAX_NODES) {
            advertisedNodes[advertisedCount++] = rId;
          }
        }
      }
      idx = comma + 1;
    }
    
    // Withdraw any routes through src that src no longer advertises
    withdrawMissingRoutes(src, advertisedNodes, advertisedCount);
  }

  // --- PROCESS DATA ---
  if (type == "D") {
    // Packet is FOR ME -> queue for display
    if (dst == NODE_ID) {
      Serial.println("RX: " + payload + " FROM " + String(src));
      
      RxDataMessage rxMsg;
      rxMsg.src = src;
      strncpy(rxMsg.payload, payload.c_str(), MAX_PAYLOAD_LEN - 1);
      rxMsg.payload[MAX_PAYLOAD_LEN - 1] = '\0';
      
      if (xQueueSend(rxDisplayQueue, &rxMsg, pdMS_TO_TICKS(100)) == pdTRUE) {
        pendingMsgCount++;
      } else {
        Serial.println("WARNING: RX queue full, message dropped!");
      }
    } 
    // Packet is FOR ME TO RELAY -> queue for relay
    else if (next == NODE_ID) {
      Serial.println("Queuing relay for Node " + String(dst));
      
      RelayRequest relay;
      relay.origSrc = src;
      relay.dst = dst;
      strncpy(relay.payload, payload.c_str(), MAX_PAYLOAD_LEN - 1);
      relay.payload[MAX_PAYLOAD_LEN - 1] = '\0';
      
      xQueueSend(relayQueue, &relay, pdMS_TO_TICKS(100));
    }
  }
}

//==================================================
// --- RECEIVER TASK (Runs on Core 0) ---
//==================================================
void receiverTask(void* parameter) {
  Serial.println("Receiver task running on Core 0");
  
  for (;;) {
    // Acquire LoRa mutex to check for incoming packets
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      int packetSize = LoRa.parsePacket();
      if (packetSize > 0) {
        // Read all raw bytes while holding the mutex
        uint8_t rawBuffer[300];
        int bytesRead = 0;
        while (LoRa.available() && bytesRead < 300) {
          rawBuffer[bytesRead++] = LoRa.read();
        }
        LoRa.receive(); // Stay in continuous RX mode after reading
        xSemaphoreGive(loraMutex);
        
        // Process outside the mutex (decryption + routing + queuing)
        processPacket(rawBuffer, bytesRead);
      } else {
        xSemaphoreGive(loraMutex);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(5)); // 5ms polling interval
  }
}

//==================================================
// --- QUEUE PROCESSING (Called from Core 1) ---
//==================================================
void processRelayQueue() {
  RelayRequest relay;
  // Process all pending relays immediately
  while (xQueueReceive(relayQueue, &relay, 0) == pdTRUE) {
    int newNextHop = getNextHop(relay.dst);
    if (newNextHop != -1) {
      displayRelaying(relay.origSrc, relay.dst, newNextHop);
      currentDisplayState = RELAYING;
      displayStateTime = millis();
      
      delay(150 + random(100)); // Anti-collision jitter
      sendRawPacket("D", relay.origSrc, relay.dst, newNextHop, String(relay.payload));
    } else {
      Serial.printf("Relay failed: No route to Node %d\n", relay.dst);
    }
  }
}

void processRxMessages() {
  // Only show received messages when user is idle
  if (currentDisplayState != IDLE) return;
  
  RxDataMessage rxMsg;
  if (xQueueReceive(rxDisplayQueue, &rxMsg, 0) == pdTRUE) {
    if (pendingMsgCount > 0) pendingMsgCount--;
    
    lastReceivedFrom = String(rxMsg.src);
    lastReceivedMsg = String(rxMsg.payload);
    displayReceived(lastReceivedFrom, lastReceivedMsg);
    currentDisplayState = RECEIVED;
    displayStateTime = millis();
  }
}

//==================================================
// --- BUTTON HANDLING ---
//==================================================
void handleShortPress(ButtonIndex button) {
  // Keyboard mode: short press navigates
  if (currentDisplayState == COMPOSING) {
    if (button == BTN_UP) moveKeyboardCursor(-1, 0);
    else if (button == BTN_DOWN) moveKeyboardCursor(1, 0);
    else if (button == BTN_LEFT) moveKeyboardCursor(0, -1);
    else if (button == BTN_RIGHT) moveKeyboardCursor(0, 1);

    displayKeyboard();
    return;
  }

  // Keep original node-selection flow on UP button only
  if (button == BTN_UP) {
    selectedTargetNode = getNextTargetNode(selectedTargetNode);
    displaySelecting();
    currentDisplayState = SELECTING;
    displayStateTime = millis();
    Serial.printf("Selected target: Node %d\n", selectedTargetNode);
  }
}

void handleLongPress(ButtonIndex button) {
  // Keyboard mode: long press on any direction key selects current key
  if (currentDisplayState == COMPOSING) {
    applyKeyboardSelection();
    return;
  }

  // Keep original node-selection flow on UP button only
  if (button == BTN_UP && selectedTargetNode != 0) {
    resetKeyboardComposer();
    currentDisplayState = COMPOSING;
    displayStateTime = millis();
    displayKeyboard();
    Serial.printf("Keyboard opened for Node %d\n", selectedTargetNode);
  }
}

void handleButton() {
  unsigned long now = millis();

  for (int i = 0; i < BTN_COUNT; i++) {
    int reading = digitalRead(BUTTON_PINS[i]);

    if (reading != lastReadingState[i]) {
      buttonLastDebounce[i] = now;
    }

    if ((now - buttonLastDebounce[i]) > DEBOUNCE_DELAY) {
      if (reading != stableButtonState[i]) {
        stableButtonState[i] = reading;

        // Button just pressed
        if (stableButtonState[i] == LOW) {
          buttonPressed[i] = true;
          longPressHandled[i] = false;
          buttonPressStart[i] = now;
        }
        // Button just released
        else if (buttonPressed[i]) {
          unsigned long pressDuration = now - buttonPressStart[i];
          buttonPressed[i] = false;

          if (!longPressHandled[i] && pressDuration < LONG_PRESS_TIME) {
            handleShortPress((ButtonIndex)i);
          }
        }
      }
    }

    lastReadingState[i] = reading;

    // Long press detect while held
    if (buttonPressed[i] && stableButtonState[i] == LOW && !longPressHandled[i]) {
      if ((now - buttonPressStart[i]) >= LONG_PRESS_TIME) {
        longPressHandled[i] = true;
        handleLongPress((ButtonIndex)i);
      }
    }
  }

  // Keep keyboard blinking cursor responsive
  if (currentDisplayState == COMPOSING && (now - lastKeyboardBlinkRefresh) > 250) {
    displayKeyboard();
    lastKeyboardBlinkRefresh = now;
  }
}

//==================================================
// --- DISPLAY STATE MANAGEMENT ---
//==================================================
void updateDisplayState() {
  unsigned long now = millis();
  
  // Return to idle after timeout (except when selecting/composing)
  if (currentDisplayState != IDLE && currentDisplayState != SELECTING && currentDisplayState != COMPOSING) {
    if (now - displayStateTime > DISPLAY_TIMEOUT) {
      currentDisplayState = IDLE;
      displayIdle();
    }
  }
  
  // Selection timeout - return to idle if no action
  if (currentDisplayState == SELECTING) {
    if (now - displayStateTime > 10000) { // 10 second timeout for selection
      currentDisplayState = IDLE;
      selectedTargetNode = 0;
      displayIdle();
    }
  }
}

//==================================================
// --- SETUP & LOOP ---
//==================================================
void setup() {
  Serial.begin(115200);
  
  // Create FreeRTOS synchronization objects
  loraMutex = xSemaphoreCreateMutex();
  routeMutex = xSemaphoreCreateMutex();
  rxDisplayQueue = xQueueCreate(MSG_QUEUE_SIZE, sizeof(RxDataMessage));
  relayQueue = xQueueCreate(RELAY_QUEUE_SIZE, sizeof(RelayRequest));
  
  // Button setup
  for (int i = 0; i < BTN_COUNT; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    int state = digitalRead(BUTTON_PINS[i]);
    stableButtonState[i] = state;
    lastReadingState[i] = state;
  }
  
  // I2C for OLED
  Wire.begin(21, 22); 
  
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(0, 20, "LoRa Mesh");
  u8g2.setFont(u8g2_font_6x10_tf);
  String nodeStr = "Node " + String(NODE_ID);
  u8g2.drawStr(0, 35, nodeStr.c_str());
  u8g2.drawStr(0, 50, "Initializing...");
  u8g2.sendBuffer();

  // LoRa setup
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa Init Failed!");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(0, 30, "LoRa FAILED!");
    u8g2.sendBuffer();
    while (1);
  }

  // Start in continuous receive mode
  LoRa.receive();
  
  Serial.printf("Node %d Ready (Dual-Core)\n", NODE_ID);
  randomSeed(analogRead(0));
  
  // Start receiver task on Core 0 (higher priority so it gets to run)
  xTaskCreatePinnedToCore(
    receiverTask,        // Task function
    "LoRaReceiver",      // Name
    8192,                // Stack size (bytes)
    NULL,                // Parameters
    2,                   // Priority (higher than loop)
    &receiverTaskHandle, // Task handle
    0                    // Core 0
  );
  Serial.println("Receiver task started on Core 0");
  
  delay(1000);
  displayIdle();
  currentDisplayState = IDLE;
}

void loop() {
  // Core 1: UI + Sending (receiving is on Core 0)
  
  // 1. Process relay queue first (time-sensitive)
  processRelayQueue();
  
  // 2. Handle button input
  handleButton();
  
  // 3. Show queued received messages when user is idle
  processRxMessages();

  unsigned long currentMillis = millis();

  // 4. Broadcast HELLO periodically
  if (currentMillis - lastHelloTime > HELLO_INTERVAL) {
    sendHello();
    lastHelloTime = currentMillis;
    
    // Check for expired routes
    expireOldRoutes();
    
    // Take thread-safe snapshot of routing table for display
    if (xSemaphoreTake(routeMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      displayRouteCount = routingTableSize;
      for (int i = 0; i < routingTableSize && i < MAX_NODES; i++) {
        displayRouteNodes[i] = routingTable[i].nodeId;
        displayRouteHops[i] = routingTable[i].nextHop;
        displayRouteDist[i] = routingTable[i].distance;
      }
      xSemaphoreGive(routeMutex);
    }
  }

  // 5. Update display state
  updateDisplayState();
  
  // 6. Refresh idle display periodically
  if (currentDisplayState == IDLE) {
    if (currentMillis - lastDisplayUpdate > 2000) {
      displayIdle();
      lastDisplayUpdate = currentMillis;
    }
  }
}