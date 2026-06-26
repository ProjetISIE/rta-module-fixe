---
lang: fr
---

<!--toc:start-->

- [Architecture Logicielle](#architecture-logicielle)
- [Paramétrage (Configuration)](#paramétrage-configuration)
  - [1. Configuration LiDAR (`main.cpp` & `Lidar.hpp`)](#1-configuration-lidar-maincpp-lidarhpp)
  - [2. Configuration BLE (`BleServer.cpp` & `rta_gatt.cpp`)](#2-configuration-ble-bleservercpp-rtagattcpp)
  - [3. Configuration ESP-NOW (`EspNowFixe.cpp`)](#3-configuration-esp-now-espnowfixecpp)
- [API Bluetooth Low Energy (BLE)](#api-bluetooth-low-energy-ble)
  - [Service : RTA Service](#service-rta-service)
  - [Caractéristique : Distance](#caractéristique-distance)
- [Spécifications Techniques](#spécifications-techniques)

<!--toc:end-->

# Rider Training Assistant (RTA) : Module Fixe

Module attaché à l’obstacle chargé de mesurer en continu sa distance au cheval
et d’émettre cette information via BLE.

## Architecture Logicielle

Le module est écrit en **C++23 moderne** et construit sur le framework ESP-IDF.
Il applique les meilleures pratiques de programmation système :

- **RAII** : Gestion automatique des ressources (UART, BLE) via des classes
  (`rta::lidar::Lidar`, `rta::ble::BleServer`).
- **Strong Typing** : Utilisation de types dédiés (`Millimeters`, `Degrees`)
  pour éviter les erreurs d'unités.
- **std::expected** : Gestion d'erreurs explicite, évitant les codes de retour
  ambigus.
- **Namespaces** : Organisation modulaire du code (`rta::ble`, `rta::lidar`,
  `rta::espnow`).

Le système repose sur plusieurs sous-systèmes fonctionnant de manière
asynchrone :

1. **LiDAR Task (FreeRTOS)** :
   - Gère la communication UART avec le capteur LiDAR (RPLiDAR).
   - Traite les trames du capteur et applique un **filtrage angulaire** (secteur
     frontal de ±22.5°) pour isoler la zone de détection pertinente.
   - Calcule la distance minimale au sein de chaque rotation complète (scan).
   - Implémente un watchdog et une logique d'auto-récupération : en cas de perte
     de données (timeout > 1s) ou d'erreurs matérielles, la distance est
     invalidée (`0xFFFF`).
2. **Communication BLE (NimBLE)** :
   - Serveur GATT gérant la pile Bluetooth Low Energy.
   - Diffuse des annonces (Advertising) en continu sous le nom `RTA_FIXE`.
   - Met à jour la caractéristique GATT `Distance` avec la valeur mesurée.
3. **Communication ESP-NOW** :
   - Diffusion broadcast ultra-rapide des distances pour synchroniser
     instantanément d'autres modules ESP (ex: module mobile).
   - Fonctionne sur l'interface Wi-Fi configurée en mode Station (STA).

## Paramétrage (Configuration)

La plupart des paramètres du système sont définis de manière statique au sein du
code source pour des raisons d'optimisation (encombrement et performances).

### 1. Configuration LiDAR (`main.cpp` & `Lidar.hpp`)

- **Port UART** : `UART_NUM_2`
- **Broches GPIO** : TX = `26`, RX = `25`
- **Vitesse (Baudrate)** : `460800 bauds` (ou bit/s, car bande de base)
- **Filtrage Angulaire** :
  - **Centre du secteur** : `0.0°` (face avant du capteur)
  - **Ouverture** : `±22.5°` (cône de détection total de 45°)
- **Plage de Mesure (Limites)** :
  - Minimum : `50 mm`
  - Maximum : `14 000 mm`
- **Timeout Watchdog** : `1 000 ms` (si aucune donnée valide n'est reçue pendant
  cette durée, la valeur bascule sur `0xFFFF`).

### 2. Configuration BLE (`BleServer.cpp` & `rta_gatt.cpp`)

- **Nom du périphérique (Device Name)** : `RTA_FIXE`
- **UUID Service RTA** : `56781234-5678-1234-1234-56789abcdef0`
- **UUID Caractéristique Distance** : `56781234-5678-1234-1234-56789abcdef1`
  (Opérations: Read, Notify)

### 3. Configuration ESP-NOW (`EspNowFixe.cpp`)

- **Interface WiFi** : Mode Station (`WIFI_MODE_STA`), mémoire gérée en RAM
  (`WIFI_STORAGE_RAM`).
- **Protocole radio** : Support 802.11b/g/n et Long Range (`WIFI_PROTOCOL_LR`).
- **Adresse MAC destination** : Broadcast (`FF:FF:FF:FF:FF:FF`).
- **Cryptage** : Désactivé (`false`).
- **Format du paquet (Payload)** :
  - **Magic string (En-tête)** : `{'R', 'T', 'A', '!'}` (4 octets)
  - **Valeur** : Entier non signé de 16 bits `uint16_t distance_mm` (2 octets)

## API Bluetooth Low Energy (BLE)

Le module expose un service GATT unique pour la lecture de la distance.

### Service : RTA Service

- **UUID** : `56781234-5678-1234-1234-56789abcdef0`

### Caractéristique : Distance

- **UUID** : `56781234-5678-1234-1234-56789abcdef1`
- **Opérations** : Read, Notify
- **Format** : Entier non signé 16 bits (`uint16_t`), Little-Endian.
- **Unité** : Millimètres (mm).
- **Valeur Spéciale (Sentinel)** : `0xFFFF` (65535) indique une absence de
  données ou une erreur matérielle du LiDAR.

## Spécifications Techniques

- **Portée de mesure** : 50mm à 14 000mm.
- **Angle de détection** : 45° frontal.
- **Fréquence de mise à jour** : Basée sur la rotation du LiDAR (~5-10 Hz).
