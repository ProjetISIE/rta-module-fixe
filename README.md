---
lang: fr
---

# Rider Training Assistant (RTA) : Module Fixe

Module attaché à l’obstacle chargé de mesurer en continu sa distance au cheval
et d’émettre cette information via BLE.

## Architecture Logicielle

Le module est construit sur l'ESP-IDF et utilise plusieurs composants clés
fonctionnant de manière asynchrone :

1. **LiDAR Task (FreeRTOS)** :
   - Gère la communication UART avec le capteur LiDAR (RPLiDAR).
   - Applique un **filtrage angulaire** (secteur frontal de ±22.5°) pour ne
     détecter que les obstacles pertinents (le cheval).
   - Calcule la distance minimale au sein de chaque scan.
   - Intègre une logique d'auto-récupération et un watchdog pour signaler les
     pertes de données.
2. **BLE Server (NimBLE)** :
   - Gère la pile Bluetooth Low Energy.
   - Diffuse des annonces (Advertising) sous le nom `RTA_FIXE`.
   - Expose les données via un serveur GATT.
3. **RTA GATT Service** :
   - Interface de données simplifiée pour la transmission de la distance.

## API Bluetooth Low Energy (BLE)

Le module expose un service GATT unique pour la lecture de la distance.

### Service : RTA Service

- **UUID** : `56781234-5678-1234-1234-56789abcdef0`

### Caractéristique : Distance

- **UUID** : `56781234-5678-1234-1234-56789abcdef1`
- **Opérations** : Read, Notify
- **Format** : Entier non signé 16 bits (`uint16_t`), Little-Endian.
- **Unité** : Millimètres (mm).
- **Valeur Spéciale (Sentinel)** : `0xFFFF` (65535) indique une absence de
  données ou une erreur matérielle du LiDAR.

## Spécifications Techniques

- **Portée de mesure** : 50mm à 14 000mm.
- **Angle de détection** : 45° frontal.
- **Fréquence de mise à jour** : Basée sur la rotation du LiDAR (~5-10 Hz).
