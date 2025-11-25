#include <esp_http_server.h>
#include <esp_log.h>
#include <sys/param.h>
#include "webserver.h"

static const char *TAG = "webserver";

// Globale Variablen für JPEG-Daten
static uint8_t *g_jpeg_data = NULL;
static size_t g_jpeg_size = 0;
static SemaphoreHandle_t g_jpeg_mutex = NULL;

// HTML-Seite
static const char* html_page = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<title>ESP32-P4 Kamera</title>"
"<style>"
"body{font-family:Arial;text-align:center;margin:20px;background:#f0f0f0}"
"#preview{max-width:800px;max-height:600px;margin:20px auto;border:2px solid #333;background:#fff}"
"button{font-size:18px;padding:15px 30px;margin:10px;cursor:pointer;border:none;border-radius:5px}"
"#btnCapture{background:#4CAF50;color:white}"
"#btnSave{background:#2196F3;color:white}"
"button:hover{opacity:0.8}"
"</style>"
"</head>"
"<body>"
"<h1>ESP32-P4 Live Kamera</h1>"
"<div><img id='preview' src='' alt='Kein Bild geladen'></div>"
"<div>"
"<button id='btnCapture' onclick='capture()'>Aufnahme</button>"
"<button id='btnSave' onclick='save()'>Speichern</button>"
"</div>"
"<script>"
"let currentImage = '';"
"function capture(){"
"  fetch('/capture').then(r=>r.blob()).then(blob=>{"
"    currentImage = URL.createObjectURL(blob);"
"    document.getElementById('preview').src = currentImage;"
"  }).catch(e=>alert('Fehler: '+e));"
"}"
"function save(){"
"  if(!currentImage){alert('Bitte zuerst Aufnahme machen!');return;}"
"  const a = document.createElement('a');"
"  a.href = currentImage;"
"  const now = new Date();"
"  const timestamp = now.getFullYear()+'-'+(now.getMonth()+1).toString().padStart(2,'0')+'-'+now.getDate().toString().padStart(2,'0')+'_'+now.getHours().toString().padStart(2,'0')+'-'+now.getMinutes().toString().padStart(2,'0')+'-'+now.getSeconds().toString().padStart(2,'0');"
"  a.download = 'CamBild_'+timestamp+'.jpg';"
"  a.click();"
"}"
"</script>"
"</body>"
"</html>";

// Handler: Haupt-Webseite
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
}

// Handler: JPEG Capture
static esp_err_t capture_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(g_jpeg_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (g_jpeg_data == NULL || g_jpeg_size == 0) {
        xSemaphoreGive(g_jpeg_mutex);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No image available");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    
    esp_err_t res = httpd_resp_send(req, (const char *)g_jpeg_data, g_jpeg_size);
    
    xSemaphoreGive(g_jpeg_mutex);
    return res;
}

static const httpd_uri_t uri_root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t uri_capture = {
    .uri       = "/capture",
    .method    = HTTP_GET,
    .handler   = capture_handler,
    .user_ctx  = NULL
};

httpd_handle_t start_webserver(void)
{
    g_jpeg_mutex = xSemaphoreCreateMutex();
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.stack_size = 8192;

    httpd_handle_t server = NULL;
    
    ESP_LOGI(TAG, "Starting HTTP Server on port %d", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_capture);
        ESP_LOGI(TAG, "Webserver started successfully");
        return server;
    }

    ESP_LOGE(TAG, "Failed to start webserver");
    return NULL;
}

void webserver_update_jpeg(const uint8_t *jpeg_data, size_t jpeg_size)
{
    if (g_jpeg_mutex == NULL) return;
    
    if (xSemaphoreTake(g_jpeg_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Kopiere JPEG-Daten (oder nutze Referenz, wenn Buffer stabil bleibt)
        static uint8_t *stored_jpeg = NULL;
        static size_t stored_size = 0;
        
        if (jpeg_size > stored_size) {
            if (stored_jpeg) free(stored_jpeg);
            stored_jpeg = malloc(jpeg_size);
            stored_size = jpeg_size;
        }
        
        if (stored_jpeg) {
            memcpy(stored_jpeg, jpeg_data, jpeg_size);
            g_jpeg_data = stored_jpeg;
            g_jpeg_size = jpeg_size;
        }
        
        xSemaphoreGive(g_jpeg_mutex);
    }
}