#include "rta_gatt.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

static const char *TAG = "RTA_GATT";

static uint16_t rta_char_float_handle;
static uint16_t rta_char_str_handle;

static float current_dist_m = -1.0f;
static char current_dist_str[16] = "DNODATA\n";

static int rta_gatt_char_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

const struct ble_gatt_svc_def rta_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(RTA_SERVICE_UUID),
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = BLE_UUID128_DECLARE(RTA_CHAR_FLOAT_UUID),
                    .access_cb = rta_gatt_char_access,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &rta_char_float_handle,
                },
                {
                    .uuid = BLE_UUID128_DECLARE(RTA_CHAR_STR_UUID),
                    .access_cb = rta_gatt_char_access,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &rta_char_str_handle,
                },
                {0}},
    },
    {0}};

static int rta_gatt_char_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
  if (attr_handle == rta_char_float_handle) {
    return os_mbuf_append(ctxt->om, &current_dist_m, sizeof(current_dist_m));
  } else if (attr_handle == rta_char_str_handle) {
    return os_mbuf_append(ctxt->om, current_dist_str, strlen(current_dist_str));
  }
  return BLE_ATT_ERR_UNLIKELY;
}

void rta_gatt_update_distance(float dist_m) {
  current_dist_m = dist_m;
  if (dist_m < 0) {
    strcpy(current_dist_str, "DNODATA\n");
  } else {
    snprintf(current_dist_str, sizeof(current_dist_str), "D%.0f\n",
             dist_m * 1000.0f);
  }

  // Notify connected clients
  ble_gatts_chr_updated(rta_char_float_handle);
  ble_gatts_chr_updated(rta_char_str_handle);
}
