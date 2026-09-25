#include "dap_lock.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static StaticSemaphore_t s_buf;
static SemaphoreHandle_t s_lock;

/* Before app_main, so no task can race the creation. */
__attribute__((constructor)) static void dap_lock_create(void)
{
    s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_buf);
}

void dap_lock(void)
{
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
}

bool dap_trylock(uint32_t timeout_ms)
{
    return xSemaphoreTakeRecursive(s_lock, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void dap_unlock(void)
{
    xSemaphoreGiveRecursive(s_lock);
}
