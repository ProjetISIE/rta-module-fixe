/*
* =============================================================================
*  RTA - Rider Training Assistant
*  PARTIE FIXE  :  RPLidar C1 (C1M1-R2)  +  ESP32-WROOM-32D
* =============================================================================
*
*  RÔLE
*  ----
*  Mesure la distance cheval → obstacle via LiDAR 2D (secteur 45° frontal).
*  Envoie la distance minimale détectée au module mobile (Nucleo-L476RG)
*  via BLE GATT — remplace l'ancien module XBee.
*
*  CÂBLAGE ESP32 ↔ LIDAR
*  ----------------------
*    Fil jaune (TX LiDAR) → GPIO16 (RX2 ESP32)
*    Fil vert  (RX LiDAR) → GPIO17 (TX2 ESP32)
*    GND commun LiDAR / ESP32
*    LiDAR alimenté 5V par alim de labo (séparée de l'USB ESP32)
*    ESP32 alimenté par USB
*
*  BLE — SERVEUR GATT
*  ------------------
*  Nom annoncé  : "RTA_FIXE"
*  Service UUID : 12345678-1234-5678-1234-56789abcdef0
*
*  Caractéristique 1 — UUID …abcdef1
*    → Distance minimale secteur 45°  (float, mètres)
*    → READ + NOTIFY  |  1 mise à jour par tour LiDAR (~10 Hz)
*
*  Caractéristique 2 — UUID …abcdef2
*    → Même valeur au format "D<mm>\n"  (ex: "D1200\n")
*    → READ + NOTIFY  |  Compatibilité ancien protocole XBee
*
*  FILTRE ANGULAIRE
*  ----------------
*  Seul le secteur ANGLE_CENTER ± ANGLE_HALF_WIDTH est traité.
*  Par défaut : 0° ± 22.5° = 45° frontal (face à l'obstacle)
*
*  AFFICHAGE SÉRIE
*  ---------------
*  1 ligne par tour LiDAR uniquement → lisible (~10 lignes/sec)
*  + statistiques toutes les 5 secondes
*
*  Board Arduino IDE : DOIT ESP32 DEVKIT V1  |  Serial : 115200 baud
* =============================================================================
*/
 
// ======================== BIBLIOTHÈQUES ========================
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
 
// ======================== PINS & BAUD ==========================
#define LIDAR_RX_PIN   16
#define LIDAR_TX_PIN   17
#define LIDAR_BAUD     460800
#define DEBUG_BAUD     115200
 
// ======================== FILTRE ANGULAIRE =====================
#define ANGLE_CENTER     0.0f    // Centre du secteur (0° = face à l'obstacle)
#define ANGLE_HALF_WIDTH 22.5f   // Demi-angle → secteur total 45°
 
// ======================== SEUILS DISTANCE ======================
#define DIST_MIN_MM    50.0f     // < 5 cm  = bruit LiDAR, ignoré
#define DIST_MAX_MM 14000.0f     // > 14 m  = hors zone utile (cahier des charges)
 
// ======================== COMMANDES RPLIDAR ====================
#define RPLIDAR_CMD_STOP       0x25
#define RPLIDAR_CMD_RESET      0x40
#define RPLIDAR_CMD_SCAN       0x20
#define RPLIDAR_CMD_GET_INFO   0x50
#define RPLIDAR_CMD_GET_HEALTH 0x52
#define RPLIDAR_ANS_SYNC1      0xA5
#define RPLIDAR_ANS_SYNC2      0x5A
 
// ======================== UUIDs BLE ============================
#define BLE_DEVICE_NAME     "RTA_FIXE"
#define BLE_SERVICE_UUID    "12345678-1234-5678-1234-56789abcdef0"
#define BLE_CHAR_FLOAT_UUID "12345678-1234-5678-1234-56789abcdef1"
#define BLE_CHAR_STR_UUID   "12345678-1234-5678-1234-56789abcdef2"
 
// ======================== VARIABLES GLOBALES ===================
HardwareSerial LidarSerial(2);
 
struct ScanPoint {
  float   angle;
  float   distance;
  uint8_t quality;
  bool    startFlag;
};
 
// BLE
BLEServer         *pBleServer    = nullptr;
BLECharacteristic *pCharFloat    = nullptr;
BLECharacteristic *pCharStr      = nullptr;
bool               bleConnected  = false;
 
// Distance minimale accumulée sur le tour en cours
float minDistMM    = DIST_MAX_MM + 1.0f;
float lastPubDistM = 0.0f;
 
// Statistiques
unsigned long cntScans     = 0;
unsigned long cntTotal     = 0;
unsigned long cntFiltered  = 0;
unsigned long tStats       = 0;
 
// =============================================================
//  CALLBACKS BLE
// =============================================================
class RtaBleCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pSrv) override {
    bleConnected = true;
    Serial.println("[BLE] >>> Client connecté <<<");
  }
  void onDisconnect(BLEServer *pSrv) override {
    bleConnected = false;
    Serial.println("[BLE] Client déconnecté — advertising relancé");
    BLEDevice::startAdvertising();
  }
};
 
// =============================================================
//  INIT BLE
// =============================================================
void initBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  pBleServer = BLEDevice::createServer();
  pBleServer->setCallbacks(new RtaBleCallbacks());
 
  BLEService *pService = pBleServer->createService(BLE_SERVICE_UUID);
 
  // Caractéristique float (mètres) — usage principal module mobile
  pCharFloat = pService->createCharacteristic(
    BLE_CHAR_FLOAT_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharFloat->addDescriptor(new BLE2902());
  float initVal = 0.0f;
  pCharFloat->setValue((uint8_t*)&initVal, sizeof(float));
 
  // Caractéristique string — format "D<mm>\n" (compatibilité XBee)
  pCharStr = pService->createCharacteristic(
    BLE_CHAR_STR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharStr->addDescriptor(new BLE2902());
  pCharStr->setValue("D0\n");
 
  pService->start();
 
  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
 
  Serial.println("[BLE] Serveur démarré — en attente de connexion...");
  Serial.println("[BLE] Nom    : " BLE_DEVICE_NAME);
  Serial.println("[BLE] Svc    : " BLE_SERVICE_UUID);
  Serial.println("[BLE] Float  : " BLE_CHAR_FLOAT_UUID);
  Serial.println("[BLE] String : " BLE_CHAR_STR_UUID);
}
 
// =============================================================
//  PUBLICATION BLE — 1 fois par tour LiDAR
// =============================================================
void publishDistance(float distMM) {
  if (distMM < DIST_MIN_MM) distMM = DIST_MIN_MM;
  if (distMM > DIST_MAX_MM) distMM = DIST_MAX_MM;
 
  float distM = distMM / 1000.0f;
  lastPubDistM = distM;
 
  // Float
  pCharFloat->setValue((uint8_t*)&distM, sizeof(float));
 
  // String format XBee "D<mm>\n"
  char buf[16];
  snprintf(buf, sizeof(buf), "D%.0f\n", distMM);
  pCharStr->setValue((uint8_t*)buf, strlen(buf));
 
  if (bleConnected) {
    pCharFloat->notify();
    pCharStr->notify();
  }
}
 
// =============================================================
//  FILTRE ANGULAIRE
// =============================================================
static inline float normalizeAngle(float a) {
  while (a <    0.0f) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}
 
bool isInSector(float angle) {
  float lo = normalizeAngle(ANGLE_CENTER - ANGLE_HALF_WIDTH);
  float hi = normalizeAngle(ANGLE_CENTER + ANGLE_HALF_WIDTH);
  angle = normalizeAngle(angle);
  if (lo <= hi) return (angle >= lo && angle <= hi);
  return (angle >= lo || angle <= hi);
}
 
// =============================================================
//  PROTOCOLE RPLIDAR
// =============================================================
void lidarSendCmd(uint8_t cmd) {
  uint8_t pkt[2] = { RPLIDAR_ANS_SYNC1, cmd };
  LidarSerial.write(pkt, 2);
  LidarSerial.flush();
}
 
bool lidarReadDescriptor(uint32_t *dLen, uint8_t *sendMode,
                          uint8_t *dType, unsigned long tms = 2000) {
  unsigned long t0 = millis();
  uint8_t d[7]; int n = 0;
  while (n < 7 && (millis() - t0) < tms)
    if (LidarSerial.available()) d[n++] = LidarSerial.read();
  if (n < 7) { Serial.println("[ERR] Timeout descriptor"); return false; }
  if (d[0] != RPLIDAR_ANS_SYNC1 || d[1] != RPLIDAR_ANS_SYNC2) {
    Serial.printf("[ERR] Sync invalide : 0x%02X 0x%02X\n", d[0], d[1]);
    return false;
  }
  uint32_t raw = d[2]|(d[3]<<8)|(d[4]<<16)|((uint32_t)d[5]<<24);
  *dLen = raw & 0x3FFFFFFF; *sendMode = (raw>>30)&0x03; *dType = d[6];
  return true;
}
 
void lidarFlush() { while (LidarSerial.available()) LidarSerial.read(); }
void lidarStop()  { lidarSendCmd(RPLIDAR_CMD_STOP);  delay(100); lidarFlush(); }
void lidarReset() { lidarSendCmd(RPLIDAR_CMD_RESET); delay(500); lidarFlush(); }
 
bool lidarGetInfo() {
  lidarFlush(); lidarSendCmd(RPLIDAR_CMD_GET_INFO);
  uint32_t dLen; uint8_t sMode, dType;
  if (!lidarReadDescriptor(&dLen, &sMode, &dType)) return false;
  uint8_t info[20]; unsigned long t0=millis(); int n=0;
  while (n<20 && (millis()-t0)<2000) if (LidarSerial.available()) info[n++]=LidarSerial.read();
  if (n<20) return false;
  Serial.printf("  LiDAR — Modèle:%d  FW:%d.%d  HW:%d\n", info[0], info[2], info[1], info[3]);
  return true;
}
 
bool lidarGetHealth() {
  lidarFlush(); lidarSendCmd(RPLIDAR_CMD_GET_HEALTH);
  uint32_t dLen; uint8_t sMode, dType;
  if (!lidarReadDescriptor(&dLen, &sMode, &dType)) return false;
  uint8_t h[3]; unsigned long t0=millis(); int n=0;
  while (n<3 && (millis()-t0)<2000) if (LidarSerial.available()) h[n++]=LidarSerial.read();
  if (n<3) return false;
  const char *st[]={"OK","WARNING","ERROR"};
  Serial.printf("  Santé : %s  (erreur: %d)\n", h[0]<3?st[h[0]]:"?", h[1]|(h[2]<<8));
  return (h[0]==0);
}
 
bool lidarStartScan() {
  lidarFlush(); lidarSendCmd(RPLIDAR_CMD_SCAN);
  uint32_t dLen; uint8_t sMode, dType;
  if (!lidarReadDescriptor(&dLen, &sMode, &dType)) return false;
  if (dLen!=5 || sMode!=1) {
    Serial.printf("[ERR] Format inattendu dLen=%d sMode=%d\n", dLen, sMode);
    return false;
  }
  Serial.printf("  Scan OK — Secteur : %.1f° → %.1f° (45°)\n",
    normalizeAngle(ANGLE_CENTER-ANGLE_HALF_WIDTH),
    normalizeAngle(ANGLE_CENTER+ANGLE_HALF_WIDTH));
  Serial.println();
  Serial.println("  Tour# |  Dist. min (m)  |  BLE");
  Serial.println("  ------|-----------------|----------");
  return true;
}
 
bool lidarReadPoint(ScanPoint *p) {
  uint8_t d[5];
  if (LidarSerial.readBytes(d,5) < 5) return false;
  if ((d[1]&0x01)!=1) return false;
  uint8_t S=d[0]&0x01, Sb=(d[0]>>1)&0x01;
  if (S==Sb) return false;
  p->startFlag=(S==1);
  p->quality=(d[0]>>2)&0x3F;
  uint16_t aRaw=((d[1]>>1)&0x7F)|((uint16_t)d[2]<<7);
  p->angle=aRaw/64.0f;
  uint16_t dRaw=d[3]|((uint16_t)d[4]<<8);
  p->distance=dRaw/4.0f;
  return true;
}
 
// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(2000);
  Serial.println("==============================================");
  Serial.println("  RTA — Rider Training Assistant");
  Serial.println("  Partie Fixe : RPLidar C1 + ESP32");
  Serial.println("==============================================\n");
 
  Serial.println("--- Init BLE ---");
  initBLE();
 
  Serial.println("\n--- Init LiDAR UART ---");
  LidarSerial.setRxBufferSize(512);
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  LidarSerial.setTimeout(200);
  Serial.printf("  GPIO%d(RX) / GPIO%d(TX) @ %d baud\n",
                LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUD);
  delay(500);
 
  Serial.println("\n--- Init LiDAR ---");
  lidarStop();  delay(200);
  lidarReset(); delay(1000);
  if (!lidarGetInfo())   Serial.println("[WARN] Infos LiDAR indisponibles");
  if (!lidarGetHealth()) Serial.println("[WARN] LiDAR signale un problème");
 
  Serial.println("\n--- Démarrage scan ---");
  if (!lidarStartScan()) {
    Serial.println("[ERREUR FATALE] Scan impossible — appuyer sur EN");
    while (true) delay(1000);
  }
  tStats = millis();
}
 
// =============================================================
//  LOOP
// =============================================================
void loop() {
 
  if (LidarSerial.available() >= 5) {
    ScanPoint pt;
    if (!lidarReadPoint(&pt)) return;
 
    cntTotal++;
 
    // --- Nouveau tour : publier la distance du tour précédent ---
    if (pt.startFlag) {
      cntScans++;
 
      if (minDistMM <= DIST_MAX_MM) {
        publishDistance(minDistMM);
 
        // Affichage : 1 ligne par tour uniquement
        Serial.printf("  %5lu |   %7.3f m     |  %s\n",
          cntScans,
          lastPubDistM,
          bleConnected ? "connecté" : "en attente");
      }
      minDistMM = DIST_MAX_MM + 1.0f;  // reset accumulateur
    }
 
    // --- Filtre angulaire 45° ---
    if (!isInSector(pt.angle)) return;
    cntFiltered++;
 
    // --- Mise à jour distance minimale du tour en cours ---
    if (pt.distance >= DIST_MIN_MM && pt.distance <= DIST_MAX_MM) {
      if (pt.distance < minDistMM) minDistMM = pt.distance;
    }
  }
 
  // --- Statistiques toutes les 5 secondes ---
  if (millis() - tStats > 5000) {
    float dt = (millis() - tStats) / 1000.0f;
    Serial.println();
    Serial.println("  ========== STATS ==========");
    Serial.printf("  Tours LiDAR      : %lu\n",      cntScans);
    Serial.printf("  Débit brut       : ~%.0f pts/s\n", cntTotal/dt);
    Serial.printf("  Débit filtré 45° : ~%.0f pts/s\n", cntFiltered/dt);
    Serial.printf("  Dernière dist.   : %.3f m\n",    lastPubDistM);
    Serial.printf("  BLE              : %s\n",
                  bleConnected ? "CLIENT CONNECTÉ" : "en attente...");
    Serial.printf("  Secteur          : %.1f° → %.1f°\n",
                  normalizeAngle(ANGLE_CENTER-ANGLE_HALF_WIDTH),
                  normalizeAngle(ANGLE_CENTER+ANGLE_HALF_WIDTH));
    Serial.println("  ===========================\n");
    cntTotal = cntFiltered = cntScans = 0;
    tStats = millis();
  }
}