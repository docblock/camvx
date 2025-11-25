#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <esp_http_server.h>
#include <stdint.h>

httpd_handle_t start_webserver(void);
void webserver_update_jpeg(const uint8_t *jpeg_data, size_t jpeg_size);

#endif