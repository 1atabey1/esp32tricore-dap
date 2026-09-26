/*
 * Data / watch-point trace endpoints (dap_mcds.h).
 *
 *   GET  /datatrace.html        the page
 *   POST /api/mcds/config       JSON config (see config_from_json)
 *   GET  /api/mcds/config       applied config + what the probe measured
 *   GET  /api/mcds/start        configure, arm, start the drain
 *   GET  /api/mcds/stop         flush, drain the rest, reset the miniMCDS
 *
 * The trace itself streams on /ws/trace (dap_web.c) as DTRP records.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "dap_mcds.h"
#include "dap_trace.h"
#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "MCDS_WEB";

extern esp_err_t check_auth(httpd_req_t *req);

static dap_mcds_config_t s_cfg;
static dap_mcds_info_t   s_info;
static bool              s_cfg_valid;

static const char *const k_source[] = { "cpu", "memslave", "lmu0" };
static const char *const k_mode[]   = { "full", "compact" };
static const char *const k_payload[] = { "addr_data", "data", "addr" };
static const char *const k_ts[]     = { "hit", "ticks", "none" };

static int pick(const cJSON *obj, const char *key, const char *const *names, int n, int dflt)
{
    const cJSON *it = cJSON_GetObjectItem(obj, key);
    if (cJSON_IsString(it)) {
        for (int i = 0; i < n; i++) {
            if (strcmp(it->valuestring, names[i]) == 0) {
                return i;
            }
        }
    }
    return dflt;
}

/* Numbers may be JSON numbers or strings ("0x7000A000"). */
static uint32_t num(const cJSON *obj, const char *key, uint32_t dflt)
{
    const cJSON *it = cJSON_GetObjectItem(obj, key);
    if (cJSON_IsNumber(it)) {
        return (uint32_t)(int64_t)it->valuedouble;
    }
    if (cJSON_IsString(it) && it->valuestring[0]) {
        return (uint32_t)strtoul(it->valuestring, NULL, 0);
    }
    return dflt;
}

static bool flag(const cJSON *obj, const char *key, bool dflt)
{
    const cJSON *it = cJSON_GetObjectItem(obj, key);
    return cJSON_IsBool(it) ? cJSON_IsTrue(it) : dflt;
}

static esp_err_t config_from_json(const char *text, dap_mcds_config_t *cfg, char *why, size_t wlen)
{
    cJSON *root = cJSON_Parse(text);
    if (!root) {
        snprintf(why, wlen, "not JSON");
        return ESP_ERR_INVALID_ARG;
    }
    dap_mcds_default_config(cfg);
    cfg->source     = pick(root, "source", k_source, 3, 0);
    cfg->cpu        = (uint8_t)num(root, "cpu", 0);
    cfg->mode       = pick(root, "mode", k_mode, 2, 0);
    cfg->payload    = pick(root, "payload", k_payload, 3, 0);
    cfg->timestamps = pick(root, "timestamps", k_ts, 3, 0);
    cfg->masters    = flag(root, "masters", false);
    cfg->dap_div    = (uint8_t)num(root, "dap_div", 0);
    cfg->wide       = flag(root, "wide", true);

    const cJSON *slots = cJSON_GetObjectItem(root, "slots");
    int enabled = 0;
    for (int j = 0; j < DAP_MCDS_SLOTS; j++) {
        const cJSON *s = cJSON_GetArrayItem(slots, j);
        dap_mcds_slot_t *d = &cfg->slot[j];
        if (!cJSON_IsObject(s)) {
            continue;
        }
        const cJSON *name = cJSON_GetObjectItem(s, "name");
        if (cJSON_IsString(name)) {
            strlcpy(d->name, name->valuestring, sizeof(d->name));
        }
        d->enabled = flag(s, "enabled", true);
        d->addr    = num(s, "addr", 0);
        d->size    = num(s, "size", 4);
        const cJSON *acc = cJSON_GetObjectItem(s, "access");
        const char *a = cJSON_IsString(acc) ? acc->valuestring : "w";
        d->rd = strchr(a, 'r') != NULL;
        d->wr = strchr(a, 'w') != NULL;
        const cJSON *v = cJSON_GetObjectItem(s, "value");
        if (cJSON_IsObject(v)) {
            d->value_en     = flag(v, "enabled", false);
            d->value_lo     = num(v, "lo", 0);
            d->value_hi     = num(v, "hi", 0xFFFFFFFFu);
            d->value_mask   = num(v, "mask", 0xFFFFFFFFu);
            d->value_signed = flag(v, "signed", false);
        }
        if (d->enabled) {
            if (d->size == 0 || (!d->rd && !d->wr)) {
                snprintf(why, wlen, "slot %d: size 0 or no access selected", j);
                cJSON_Delete(root);
                return ESP_ERR_INVALID_ARG;
            }
            enabled++;
        }
    }
    cJSON_Delete(root);
    if (!enabled) {
        snprintf(why, wlen, "no enabled slot");
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static cJSON *config_to_json(const dap_mcds_config_t *cfg)
{
    char hex[16];
    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "source", k_source[cfg->source]);
    cJSON_AddNumberToObject(root, "cpu", cfg->cpu);
    cJSON_AddStringToObject(root, "mode", k_mode[cfg->mode]);
    cJSON_AddStringToObject(root, "payload", k_payload[cfg->payload]);
    cJSON_AddStringToObject(root, "timestamps", k_ts[cfg->timestamps]);
    cJSON_AddBoolToObject(root, "masters", cfg->masters);
    cJSON_AddNumberToObject(root, "dap_div", cfg->dap_div);
    cJSON_AddBoolToObject(root, "wide", cfg->wide);
    cJSON *slots = cJSON_AddArrayToObject(root, "slots");
    for (int j = 0; j < DAP_MCDS_SLOTS; j++) {
        const dap_mcds_slot_t *s = &cfg->slot[j];
        cJSON *o = cJSON_CreateObject();
        cJSON_AddBoolToObject(o, "enabled", s->enabled);
        cJSON_AddStringToObject(o, "name", s->name);
        snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)s->addr);
        cJSON_AddStringToObject(o, "addr", hex);
        cJSON_AddNumberToObject(o, "size", s->size);
        cJSON_AddStringToObject(o, "access", s->rd && s->wr ? "rw" : s->rd ? "r" : "w");
        cJSON *v = cJSON_AddObjectToObject(o, "value");
        cJSON_AddBoolToObject(v, "enabled", s->value_en);
        snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)s->value_lo);
        cJSON_AddStringToObject(v, "lo", hex);
        snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)s->value_hi);
        cJSON_AddStringToObject(v, "hi", hex);
        snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)s->value_mask);
        cJSON_AddStringToObject(v, "mask", hex);
        cJSON_AddBoolToObject(v, "signed", s->value_signed);
        cJSON_AddItemToArray(slots, o);
    }
    return root;
}

static esp_err_t send_json(httpd_req_t *req, cJSON *root)
{
    char *text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, text ? text : "{}");
    free(text);
    return ESP_OK;
}

static esp_err_t config_post_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    char buf[2048];
    const int len = req->content_len;
    if (len <= 0 || len >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "config too large or empty");
        return ESP_OK;
    }
    int got = 0;
    while (got < len) {
        const int r = httpd_req_recv(req, buf + got, len - got);
        if (r <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "upload stopped early");
            return ESP_OK;
        }
        got += r;
    }
    buf[got] = '\0';

    char why[96] = "";
    dap_mcds_config_t cfg;
    if (config_from_json(buf, &cfg, why, sizeof(why)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, why);
        return ESP_OK;
    }
    s_cfg = cfg;
    s_cfg_valid = true;
    return send_json(req, config_to_json(&s_cfg));
}

static esp_err_t config_get_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    if (!s_cfg_valid) {
        dap_mcds_default_config(&s_cfg);
    }
    cJSON *root = config_to_json(&s_cfg);
    cJSON_AddBoolToObject(root, "running", dap_mcds_running());
    cJSON_AddNumberToObject(root, "emu_hz", s_info.emu_hz);
    cJSON_AddNumberToObject(root, "tsu_start", s_info.tsu_start);
    cJSON_AddBoolToObject(root, "wide_active", s_info.wide);
    cJSON_AddNumberToObject(root, "format", 1);

    /* Drain counters, so a client can show progress without the text endpoint. */
    dap_trace_stats_t st;
    dap_trace_get_stats(&st);
    cJSON *stats = cJSON_AddObjectToObject(root, "stats");
    cJSON_AddNumberToObject(stats, "paragraphs", st.paragraphs);
    cJSON_AddNumberToObject(stats, "bytes", st.bytes);
    cJSON_AddNumberToObject(stats, "lost", st.lost);
    cJSON_AddNumberToObject(stats, "laps", st.laps);
    cJSON_AddNumberToObject(stats, "overruns", st.overruns);
    cJSON_AddNumberToObject(stats, "read_errors", st.read_errors);
    cJSON_AddNumberToObject(stats, "queue_free", st.queue_free);
    cJSON_AddNumberToObject(stats, "queue_dropped", st.queue_dropped);
    cJSON_AddNumberToObject(stats, "poll_us_max", st.poll_us_max);
    return send_json(req, root);
}

static esp_err_t start_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    if (!s_cfg_valid) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "POST /api/mcds/config first");
        return ESP_OK;
    }
    const esp_err_t err = dap_mcds_start(&s_cfg, &s_info);
    char line[128];
    snprintf(line, sizeof(line), "%s emu_hz=%lu wide=%d\n",
             err == ESP_OK ? "started" : esp_err_to_name(err), (unsigned long)s_info.emu_hz,
             s_info.wide ? 1 : 0);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
    }
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, line);
    return ESP_OK;
}

static esp_err_t stop_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;

    dap_mcds_stop();
    dap_trace_stats_t st;
    dap_trace_get_stats(&st);
    char line[160];
    snprintf(line, sizeof(line), "stopped paragraphs=%lu lost=%lu queue_dropped=%lu\n",
             (unsigned long)st.paragraphs, (unsigned long)st.lost,
             (unsigned long)st.queue_dropped);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, line);
    return ESP_OK;
}

static esp_err_t page_handler(httpd_req_t *req)
{
    if (check_auth(req) != ESP_OK) return ESP_OK;
    extern const unsigned char page_start[] asm("_binary_datatrace_html_start");
    extern const unsigned char page_end[]   asm("_binary_datatrace_html_end");

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)page_start, page_end - page_start);
}

void mcds_web_register(httpd_handle_t server)
{
    static const httpd_uri_t uris[] = {
        { .uri = "/datatrace.html",  .method = HTTP_GET,  .handler = page_handler },
        { .uri = "/api/mcds/config", .method = HTTP_POST, .handler = config_post_handler },
        { .uri = "/api/mcds/config", .method = HTTP_GET,  .handler = config_get_handler },
        { .uri = "/api/mcds/start",  .method = HTTP_GET,  .handler = start_handler },
        { .uri = "/api/mcds/stop",   .method = HTTP_GET,  .handler = stop_handler },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        if (httpd_register_uri_handler(server, &uris[i]) != ESP_OK) {
            ESP_LOGE(TAG, "could not register %s", uris[i].uri);
        }
    }
}
