/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_cache.h"
#include "driver/i2c_master.h"
#include "driver/isp.h"
#include "driver/jpeg_encode.h" // WICHTIG: JPEG Header
#include "driver/ppa.h"         // WICHTIG: PPA Header
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "example_sensor_init.h"
#include "example_config.h"
#include "esp_ldo_regulator.h"

#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "webserver.h" 

// TFLite Stuff
#include "image_classifier.h"
#include "image_preprocessing.h"

static uint8_t* classifier_input = NULL;   // 224x224x3 für TFLite
// Ende TFLite Stuff

static const char *TAG = "cam_csi";

//Achtung das Konfigsystem ist etwas seltsam, da es in Kconfig.projbuild definiert wird
//und dann automatisch in sdkconfig.h mit dem Zusatz CONFIG_ eingebunden wird
#define EXAMPLE_MIPI_CSI_VRES      CONFIG_EXAMPLE_MIPI_CSI_VRES
#define EXAMPLE_MIPI_CSI_HRES      CONFIG_EXAMPLE_MIPI_CSI_HRES

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

#define NUM_FRAME_BUFFERS 2

// Skaliertes Bild
#define SCALED_WIDTH  224
#define SCALED_HEIGHT 224

// JPEG Buffer Größe (Schätzung: 1/5 von RGB565 sollte reichen, aber sicherheitshalber großzügig)
#define JPEG_BUFFER_SIZE (SCALED_WIDTH * SCALED_HEIGHT / 3)

typedef struct {
    void *frame_buffers[NUM_FRAME_BUFFERS];
    size_t buffer_size;
    int current_buffer_idx;
} camera_context_t;



static camera_context_t cam_ctx = {0};
static QueueHandle_t xFrameQueue = NULL;
static jpeg_encoder_handle_t jpeg_handle = NULL; // Handle für den Encoder
static ppa_client_handle_t ppa_srm_handle = NULL;  // NEU: PPA Handle
static uint8_t *scaled_buffer = NULL;              // NEU: Buffer für skaliertes Bild
// WICHTIG: volatile hinzufügen!
static volatile bool ready_to_process = true;
static volatile bool need_new_buffer = false;


typedef struct {
    void *buffer;
    size_t len;
    int buffer_idx;
} frame_event_t;

// --- Callbacks bleiben gleich ---
static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    //Diese Funktion wird vom Kamera Treiber im ISR Kontext aufgerufen, wenn ein Frame fertig aufgenommen wurde
    //Erst wenn diese Funktion zurückkehrt, wird s_camera_get_new_vb aufgerufen, um den nächsten Buffer zuzuweisen

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    frame_event_t evt;
    evt.buffer = trans->buffer;
    evt.len = trans->buflen;
    //ready_to_process wird im frame_processing_task zurückgesetzt, wenn der Frame verarbeitet wurde
    if (ready_to_process){
        need_new_buffer = true;
        ready_to_process = false;
        xQueueSendFromISR(xFrameQueue, &evt, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }
    need_new_buffer = false;
    return false;
}

static bool s_camera_get_new_vb(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    // Wenn das letzte Bild fertig ist, wechseln wir zum nächsten Buffer
    // Wenn das letzte Bild noch nicht fertig ist, wird der gleiche Buffer wiederverwendet
    // Damit werden alle Bilder verworfen, die aufgenommen wurden, während der Prozessor noch beschäftigt war.

    camera_context_t *ctx = (camera_context_t *)user_data;
    if(need_new_buffer)
        ctx->current_buffer_idx = (ctx->current_buffer_idx + 1) % NUM_FRAME_BUFFERS;
    trans->buffer = ctx->frame_buffers[ctx->current_buffer_idx];
    trans->buflen = ctx->buffer_size;
    return false;
}

// NEU: PPA Skalierungsfunktion mit Spiegelung
static esp_err_t scale_image_with_ppa(const uint8_t *src, uint8_t *dst)
{
    // 1. Cache Sync für Input
    size_t src_size = EXAMPLE_MIPI_CSI_HRES * EXAMPLE_MIPI_CSI_VRES * 2;
    esp_cache_msync((void*)src, src_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    // Dynamische Berechnung des Skalierungsfaktors
    float scale_x = (float)SCALED_WIDTH / (float)EXAMPLE_MIPI_CSI_HRES;
    float scale_y = (float)SCALED_HEIGHT / (float)EXAMPLE_MIPI_CSI_VRES;

    ppa_srm_oper_config_t srm_config = {
        .in = {
            .buffer = src,
            .pic_w = EXAMPLE_MIPI_CSI_HRES,
            .pic_h = EXAMPLE_MIPI_CSI_VRES,
            .block_w = EXAMPLE_MIPI_CSI_HRES,
            .block_h = EXAMPLE_MIPI_CSI_VRES,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = dst,
            // KORREKTUR: RGB888 braucht Faktor 3, nicht 2!
            .buffer_size = SCALED_WIDTH * SCALED_HEIGHT * 3, 
            .pic_w = SCALED_WIDTH,
            .pic_h = SCALED_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB888, // Hier fordern Sie RGB888 an
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = scale_x,
        .scale_y = scale_y, 
        .rgb_swap = 0,
        .byte_swap = 0,
        .mode = PPA_TRANS_MODE_BLOCKING,
        // NEU: Spiegelung aktivieren
        .mirror_x = true,   // Horizontal spiegeln (links <-> rechts)
        .mirror_y = false,  // Vertikal spiegeln (oben <-> unten) - falls nötig
    };

    esp_err_t ret = ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
    
    if (ret == ESP_OK) {
        // KORREKTUR: Auch hier Faktor 3 für Cache Sync
        size_t dst_size = SCALED_WIDTH * SCALED_HEIGHT * 3;
        esp_cache_msync((void*)dst, dst_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    } else {
        ESP_LOGE(TAG, "PPA scaling failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// --- Processing Task mit JPEG Encoding ---
void frame_processing_task(void *arg) {
    frame_event_t evt;
    
    // Buffer für das fertige JPEG Bild allokieren
    uint8_t *jpeg_buffer = heap_caps_aligned_calloc(64, 1, JPEG_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    /*
    size_t classifier_input_size = TM_INPUT_WIDTH * TM_INPUT_HEIGHT * TM_INPUT_CHANNELS;
    classifier_input = heap_caps_malloc(classifier_input_size, MALLOC_CAP_SPIRAM);
    if (!jpeg_buffer|| !classifier_input) {
        ESP_LOGE(TAG, "Failed to alloc JPEG buffer");
        vTaskDelete(NULL);
    }
    */
     

    while (1) {
        if (xQueueReceive(xFrameQueue, &evt, portMAX_DELAY) == pdTRUE) {
            
            ESP_LOGI(TAG, "Encoding Frame at %p...", evt.buffer);

            // NEU: Bild skalieren
            // NEU: Bild mit PPA skalieren (800x800 -> 400x400)
            esp_err_t scale_ret = scale_image_with_ppa((const uint8_t *)evt.buffer, scaled_buffer);
            if (scale_ret != ESP_OK) {
                ESP_LOGE(TAG, "Scaling failed, skipping frame");
                ready_to_process = true;
                continue;
            }
            ESP_LOGI(TAG, "Image scaled from %dx%d to %dx%d", EXAMPLE_MIPI_CSI_HRES, EXAMPLE_MIPI_CSI_VRES, SCALED_WIDTH, SCALED_HEIGHT);

            // 4. Klassifikation durchführen
            classification_result_t result;
            //if (classifier_run(classifier_input, &result)) {
            if (classifier_run(scaled_buffer, &result)) {
                ESP_LOGI(TAG, "Classification: %s (%.1f%%)", 
                         result.class_name, result.confidence * 100.0f);
                
                // Optional: Ergebnis an Webserver senden
                // webserver_update_classification(&result);
            }

            
            // 1. Config für dieses Bild
            jpeg_encode_engine_cfg_t encode_eng_cfg = {
                .intr_priority = 0,
            };

            jpeg_encode_memory_alloc_cfg_t rx_mem_cfg = {
                .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
            };

            jpeg_encode_memory_alloc_cfg_t tx_mem_cfg = {
                .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER,
            };

            // 2. Bild-Parameter setzen
            jpeg_encode_cfg_t enc_config = {
                .src_type = JPEG_ENCODE_IN_FORMAT_RGB888,
                .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
                .image_quality = 80,
                .width = SCALED_WIDTH,
                .height = SCALED_HEIGHT,
            };

            // 3. Encoding durchführen (One-Shot Modus)
            // Der Treiber kümmert sich um DMA und Cache Sync
            
            // ÄNDERUNG: uint32_t statt size_t verwenden
            uint32_t jpeg_size = 0; 
            
            // KORREKTUR: Faktor 3 für RGB888
            size_t scaled_size = SCALED_WIDTH * SCALED_HEIGHT * 3;
            
            esp_err_t ret = jpeg_encoder_process(jpeg_handle, &enc_config, 
                                                 scaled_buffer, scaled_size,
                                                 jpeg_buffer, JPEG_BUFFER_SIZE, 
                                                 &jpeg_size);

            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "JPEG Encoded! Size: %u bytes (Ratio: 1:%.1f)", 
                         jpeg_size, (float)evt.len / jpeg_size);
                
                // NEU: JPEG an Webserver übergeben
                webserver_update_jpeg(jpeg_buffer, jpeg_size);
                
            } else {
                ESP_LOGE(TAG, "JPEG Encode failed: %s", esp_err_to_name(ret));
            }

            //künstliche Verzögerung zum Testen 
            //vTaskDelay(pdMS_TO_TICKS(100)); 

        }
        ready_to_process = true;
    }
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
        /* If you only want to open more logs in the wifi module, you need to make the max level greater than the default level,
         * and call esp_log_level_set() before esp_wifi_init() to improve the log level of the wifi module. */
        esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    // NEU: Webserver starten (nach WiFi-Verbindung!)
    start_webserver();

    ret = ESP_FAIL;

    //mipi ldo
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = CONFIG_EXAMPLE_USED_LDO_CHAN_ID,
        .voltage_mv = CONFIG_EXAMPLE_USED_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));

    // --------------------------------------------------------
    // NEU: PPA Engine initialisieren
    // --------------------------------------------------------
    ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_SRM,  // Scale-Rotate-Mirror
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_client_config, &ppa_srm_handle));
    ESP_LOGI(TAG, "PPA Engine initialized");

    // NEU: Buffer für skaliertes Bild allokieren
    // KORREKTUR: Faktor 3 für RGB888
    size_t scaled_buffer_size = SCALED_WIDTH * SCALED_HEIGHT * 3; 
    scaled_buffer = heap_caps_aligned_calloc(64, 1, scaled_buffer_size, MALLOC_CAP_SPIRAM);
    if (!scaled_buffer) {
        ESP_LOGE(TAG, "Failed to alloc scaled buffer");
        return;
    }
    ESP_LOGI(TAG, "Scaled buffer allocated: %d bytes", scaled_buffer_size);
    // --------------------------------------------------------
    
    
    //---------------Frame Buffer Allocation------------------//
    cam_ctx.buffer_size = EXAMPLE_MIPI_CSI_HRES * EXAMPLE_MIPI_CSI_VRES * EXAMPLE_RGB565_BITS_PER_PIXEL / 8;
    cam_ctx.current_buffer_idx = 0;

    for (int i = 0; i < NUM_FRAME_BUFFERS; i++) {
        cam_ctx.frame_buffers[i] = heap_caps_aligned_calloc(64, 1, cam_ctx.buffer_size, MALLOC_CAP_SPIRAM);
        if (cam_ctx.frame_buffers[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate frame buffer %d", i);
            return;
        }
        // Init to white
        memset(cam_ctx.frame_buffers[i], 0xFF, cam_ctx.buffer_size);
        esp_cache_msync((void *)cam_ctx.frame_buffers[i], cam_ctx.buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        
        ESP_LOGD(TAG, "Allocated buffer[%d]: %p", i, cam_ctx.frame_buffers[i]);
    }

    //--------Camera Sensor and SCCB Init-----------//
    i2c_master_bus_handle_t i2c_bus_handle = NULL;
    example_sensor_init(I2C_NUM_0, &i2c_bus_handle);

    //---------------CSI Init------------------//
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = EXAMPLE_MIPI_CSI_HRES,
        .v_res = EXAMPLE_MIPI_CSI_VRES,
        .lane_bit_rate_mbps = EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = 2,
        .byte_swap_en = false,
        .queue_items = 2,
    };
    esp_cam_ctlr_handle_t cam_handle = NULL;
    ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "csi init fail[%d]", ret);
        return;
    }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = s_camera_get_new_vb,
        .on_trans_finished = s_camera_get_finished_trans,
    };
    
    // WICHTIG: Wir übergeben jetzt unsere cam_ctx Struktur als user_data
    if (esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, &cam_ctx) != ESP_OK) {
        ESP_LOGE(TAG, "ops register fail");
        return;
    }

    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle));

    //---------------ISP Init------------------//
    isp_proc_handle_t isp_proc = NULL;
    esp_isp_processor_cfg_t isp_config = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = EXAMPLE_MIPI_CSI_HRES,
        .v_res = EXAMPLE_MIPI_CSI_VRES,
    };
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc));
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc));

    // --------------------------------------------------------
    // NEU: JPEG Encoder Initialisierung
    // --------------------------------------------------------
    jpeg_encode_engine_cfg_t jpeg_eng_cfg = {
        .intr_priority = 0, // Default
        .timeout_ms = 40,   // Timeout für Encoding
    };
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&jpeg_eng_cfg, &jpeg_handle));
    // --------------------------------------------------------

    // NEU: Klassifikator initialisieren (VOR dem Task-Start!)
    if (!classifier_init()) {
        ESP_LOGE(TAG, "Classifier init failed!");
        // Optional: Weiter ohne Klassifikation
    }

    vTaskDelay(pdMS_TO_TICKS(10000));

    xFrameQueue = xQueueCreate(5, sizeof(frame_event_t));
    xTaskCreatePinnedToCore(frame_processing_task, "frame_proc", 8192, NULL, 5, NULL, 1); // Stack erhöht!

    esp_cam_ctlr_trans_t trans_init = {
        .buffer = cam_ctx.frame_buffers[0],
        .buflen = cam_ctx.buffer_size,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_receive(cam_handle, &trans_init, 0)); 
    
    ESP_LOGI(TAG, "Starting camera...");
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam_handle));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "in main loop");
    }
}
