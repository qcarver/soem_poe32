#include "acquire.h"
#include "esp_log.h"

static const char *TAG = "acquire";

extern "C" {

int acquire_init(void)
{
    ESP_LOGI(TAG, "Initializing data acquisition");
    // TODO: Add initialization logic
    return 0;
}

int acquire_start(void)
{
    ESP_LOGI(TAG, "Starting data acquisition");
    // TODO: Add acquisition start logic
    return 0;
}

void acquire_stop(void)
{
    ESP_LOGI(TAG, "Stopping data acquisition");
    // TODO: Add acquisition stop logic
}

} // extern "C"
