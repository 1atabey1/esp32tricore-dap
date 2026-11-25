#pragma once

#include "esp_err.h"

/*
 * Out-of-band control server: a second, minimal HTTP server whose only job is
 * to reboot the probe when the main web server is wedged.
 *
 * It is separate from web_server.c on purpose.  esp_http_server runs one task
 * per instance and serves requests one at a time, so a stuck handler (a long
 * capture, an upload, a flash operation) or a full socket table on port 80
 * takes every endpoint on that instance off the air - including any reset
 * endpoint registered there.  This instance has its own task, its own socket
 * budget and a single handler that returns immediately.
 *
 *   POST http://<probe>:CTRL_SERVER_PORT/api/reboot   (HTTP Basic auth)
 *
 * Start it before the main web server so it is up even if that fails.
 */
#define CTRL_SERVER_PORT 8081

esp_err_t ctrl_server_start(void);
