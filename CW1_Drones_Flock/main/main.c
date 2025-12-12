/****************************************************************************
 * COMP0221 Coursework 1 - FINAL VERSION (MATCHING IMAGE SETUP)
 * * --- CONFIGURATION (MATCHING YOUR IMAGE) ---
 * Edit the values below to match the 'lora.begin' parameters you wanted.
 ****************************************************************************/

// 1. Frequency
#define LORA_FREQ           868.2   // MHz

// 2. Bandwidth (125, 250, or 500)
#define LORA_BW             500     // kHz

// 3. Spreading Factor (7-12)
#define LORA_SF             7

// 4. Coding Rate (5=4/5, 6=4/6, 7=4/7, 8=4/8)
#define LORA_CR             7       

// 5. Sync Word (0x12 for private, 0x34 for LoRaWAN)
#define LORA_SYNCWORD       0x12    

// 6. Output Power (2 to 17)
#define LORA_POWER          14      // dBm

// 7. Preamble Length
#define LORA_PREAMBLE       10      

// --- ATTACK CONTROL ---
#define ENABLE_FLOOD_ATTACK     0   // Set to 1 to enable
#define ENABLE_REPLAY_ATTACK    0   // Set to 1 to enable

/****************************************************************************
 * INTERNAL REGISTER CALCULATIONS (DO NOT EDIT)
 * This maps your simple config above to the SX1276 hex registers.
 ****************************************************************************/
#if LORA_BW == 125
    #define _REG_BW 0x70
#elif LORA_BW == 250
    #define _REG_BW 0x80
#elif LORA_BW == 500
    #define _REG_BW 0x90
#else
    #error "Invalid LORA_BW"
#endif

#if LORA_CR == 5
    #define _REG_CR 0x02
#elif LORA_CR == 6
    #define _REG_CR 0x04
#elif LORA_CR == 7
    #define _REG_CR 0x06
#elif LORA_CR == 8
    #define _REG_CR 0x08
#else
    #error "Invalid LORA_CR"
#endif

#define _REG_SF (LORA_SF << 4)
#define _REG_POWER_VAL (0x80 | 0x70 | (LORA_POWER - 2))

/****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "esp_random.h"

#include "nvs_flash.h"
#include "esp_eap_client.h"
#include "mqtt_client.h"
#include "mbedtls/cmac.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "CW1";

typedef struct {
    int64_t count;
    int64_t sum;
    int64_t sum_sq;
    int64_t min;
    int64_t max;
} stats_t;

static inline void perf_stats_init(stats_t *s)
{
    s->count = 0;
    s->sum   = 0;
    s->sum_sq = 0;
    s->min   = INT64_MAX;
    s->max   = INT64_MIN;
}

static inline void perf_stats_update(stats_t *s, int64_t v)
{
    s->count++;
    s->sum += v;
    s->sum_sq += v * v;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
}

static inline int64_t perf_stats_mean(const stats_t *s)
{
    return (s->count > 0) ? (s->sum / s->count) : 0;
}



// --- LORA PINS ---
#define LORA_SCK    5
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_SS     18
#define LORA_RST    14  // 23 for Heltec V3
#define LORA_DIO0   26

static spi_device_handle_t lora_spi;

static void lora_write_reg(uint8_t reg, uint8_t val) {
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,
        .tx_data = { (uint8_t)(reg | 0x80), val }
    };
    spi_device_transmit(lora_spi, &t);
}

static uint8_t lora_read_reg(uint8_t reg) {
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,
        .tx_data = { (uint8_t)(reg & 0x7F), 0 }
    };
    spi_device_transmit(lora_spi, &t);
    return t.rx_data[1];
}

static void lora_reset() {
    gpio_set_level(LORA_RST, 0); vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LORA_RST, 1); vTaskDelay(pdMS_TO_TICKS(20));
}

// Fixed Mode Helpers (Bit 7 = 1)
static void lora_set_mode_sleep(void)   { lora_write_reg(0x01, 0x80); }
static void lora_set_mode_standby(void) { lora_write_reg(0x01, 0x81); }
static void lora_set_mode_tx(void)      { lora_write_reg(0x01, 0x83); }
static void lora_set_mode_rx_cont(void) { lora_write_reg(0x01, 0x85); }

/* --- Initialization using your Image Parameters --- */
static void lora_init() {
    spi_bus_config_t bus = {
        .miso_io_num = LORA_MISO, .mosi_io_num = LORA_MOSI,
        .sclk_io_num = LORA_SCK, .quadwp_io_num = -1, .quadhd_io_num = -1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 4000000, .mode = 0, .spics_io_num = LORA_SS, .queue_size = 1
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &lora_spi));

    gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(LORA_DIO0, GPIO_MODE_INPUT);

    lora_reset();
    
    // Explicit LoRa Mode
    lora_set_mode_sleep();      
    lora_write_reg(0x01, 0x88); 
    lora_set_mode_sleep();      
    lora_set_mode_standby();    

    // --- APPLY CONFIG FROM IMAGE ---
    
    // BW & CR (RegModemConfig1)
    lora_write_reg(0x1D, _REG_BW | _REG_CR);

    // SF & CRC (RegModemConfig2)
    lora_write_reg(0x1E, _REG_SF | 0x04);

    // AGC (RegModemConfig3 - matches 'true' in image for LNA)
    lora_write_reg(0x26, 0x04);

    // Preamble (RegPreamble)
    lora_write_reg(0x20, 0x00); 
    lora_write_reg(0x21, (uint8_t)LORA_PREAMBLE);

    // Sync Word
    lora_write_reg(0x39, LORA_SYNCWORD);

    // Power (RegPaConfig)
    lora_write_reg(0x09, _REG_POWER_VAL);

    // Reset FIFO
    lora_write_reg(0x0E, 0x00); lora_write_reg(0x0F, 0x00); 

    
    lora_set_mode_rx_cont();
}

static void lora_set_frequency(long freq) {
    uint64_t frf = ((uint64_t)freq << 19) / 32000000ULL;
    lora_write_reg(0x06, (frf >> 16) & 0xFF);
    lora_write_reg(0x07, (frf >> 8) & 0xFF);
    lora_write_reg(0x08, frf & 0xFF);
}

static void lora_enable_crc() {
    lora_write_reg(0x1E, lora_read_reg(0x1E) | 0x04);
}

static void lora_send_packet(uint8_t *data, int len) {
    lora_set_mode_standby();
    lora_write_reg(0x0D, 0x00);
    for (int i = 0; i < len; i++) lora_write_reg(0x00, data[i]);
    lora_write_reg(0x22, len);
    lora_set_mode_tx();
    int t = 100;
    while (!(lora_read_reg(0x12) & 0x08) && t-- > 0) vTaskDelay(pdMS_TO_TICKS(2));
    lora_write_reg(0x12, 0xFF);
    lora_set_mode_rx_cont(); 
}

static int lora_receive_packet(uint8_t *buf, int max_len) {
    uint8_t irq = lora_read_reg(0x12);
    if (!(irq & 0x40)) return -1;
    lora_write_reg(0x12, 0xFF);
    if (irq & 0x20) return -1;
    int len = lora_read_reg(0x13);
    if (len > max_len) len = max_len;
    uint8_t fifo_addr = lora_read_reg(0x10);
    lora_write_reg(0x0D, fifo_addr);
    for (int i = 0; i < len; i++) buf[i] = lora_read_reg(0x00);
    return len;
}

// --- WIFI / MQTT / PHYSICS CONFIG ---
#define TEAM_ID 1
#define WIFI_MODE_EDUROAM 1 
#define HOME_WIFI_SSID     "xxxx"
#define HOME_WIFI_PASSWORD "xxxxx"
#define WIFI_SSID_EDU      "eduroam" 
#define EAP_ID             "xxxx@ucl.ac.uk"
#define EAP_USERNAME       "xxxx@ucl.ac.uk"
#define EAP_PASSWORD       "xxxxxxxxxx"
#define MQTT_URI   "mqtt://broker.emqx.io:1883"
#define MQTT_TOPIC "flocksim"

#define BOX_SIZE_MM    100000
#define NEI_TIMEOUT_MS 3000
#define K_COH   0.01f
#define K_ALIGN 0.08f
#define K_SEP   1.0f
#define SEP_RADIUS_MM 4000.0f
#define MAX_SPEED_XY_MM_S   6000.0f   
#define MAX_SPEED_Z_MM_S    3000.0f  
#define MIN_SPEED_XY_MM_S   1000.0f   

static const uint8_t CMAC_KEY[16] = {
    0x2B, 0x7E, 0x15, 0x16, 0x22, 0xA0, 0xD2, 0xA6,
    0xAC, 0xF7, 0x19, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

// --- DATA STRUCTURES ---
typedef struct {
    uint8_t mac[6]; uint32_t seq_number;
    int32_t px, py, pz; int32_t vx, vy, vz;
    uint16_t yaw_cd;
} local_state_t;

typedef struct {
    bool active; uint8_t mac[6]; uint32_t last_heard;
    int32_t px, py, pz; int32_t vx, vy, vz;
} neighbour_t;

typedef struct __attribute__((packed)) {
    uint8_t     version; uint8_t     team_id; uint8_t     node_id[6];
    uint16_t    seq_number; uint32_t    ts_s; uint16_t    ts_ms;      
    uint32_t    x_mm; uint32_t    y_mm; uint32_t    z_mm;       
    int32_t     vx_mm_s; int32_t     vy_mm_s; int32_t     vz_mm_s;
    uint16_t    yaw_cd; uint8_t     mac_tag[4];
} comp0221_packet_t;

// --- GLOBALS ---
static local_state_t g_self;
static neighbour_t g_nei[6];
static SemaphoreHandle_t lock_state, lock_nei, lock_lora;
static EventGroupHandle_t wifi_events;
static esp_mqtt_client_handle_t mqtt_client;

// ---- Performance statistics ----
static stats_t tx_jitter_stats;
static stats_t rx_interval_stats;
static stats_t rssi_stats;
static stats_t snr_stats;


static volatile bool g_attack_replay_enabled = (ENABLE_REPLAY_ATTACK == 1);
static volatile bool g_attack_flood_enabled = (ENABLE_FLOOD_ATTACK == 1);
static comp0221_packet_t g_last_valid_frame; 
static bool g_has_valid_frame = false;
#define WIFI_CONNECTED_BIT BIT0

static uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000ULL); }

static void compute_cmac4(const uint8_t *buf, size_t len, uint8_t out[4]) {
    uint8_t full[16];
    mbedtls_cipher_context_t ctx;
    mbedtls_cipher_init(&ctx);
    mbedtls_cipher_setup(&ctx, mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB));
    mbedtls_cipher_cmac_starts(&ctx, CMAC_KEY, 128);
    mbedtls_cipher_cmac_update(&ctx, buf, len);
    mbedtls_cipher_cmac_finish(&ctx, full);
    mbedtls_cipher_free(&ctx);
    memcpy(out, full + 12, 4);
}

// --- TASKS ---
static void init_sntp() {
    ESP_LOGI(TAG, "Starting SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    for(int i=0; i<10; i++) {
        if(esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
            ESP_LOGI(TAG, "SNTP Sync Complete"); break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void task_physics(void *arg) {
    TickType_t last = xTaskGetTickCount();
    const float dt = 0.02f;
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
        xSemaphoreTake(lock_state, portMAX_DELAY);
        g_self.px += (int32_t)(g_self.vx * dt); g_self.py += (int32_t)(g_self.vy * dt); g_self.pz += (int32_t)(g_self.vz * dt);
        
        if(g_self.px < 0 || g_self.px > BOX_SIZE_MM) { g_self.vx = -g_self.vx; }
        if(g_self.py < 0 || g_self.py > BOX_SIZE_MM) { g_self.vy = -g_self.vy; }
        if(g_self.pz < 0 || g_self.pz > BOX_SIZE_MM) { g_self.vz = -g_self.vz; }
        
        if(g_self.px < 0) { g_self.px=0; } if(g_self.py < 0) { g_self.py=0; } if(g_self.pz < 0) { g_self.pz=0; }
        if(g_self.px > BOX_SIZE_MM) { g_self.px=BOX_SIZE_MM; } 
        if(g_self.py > BOX_SIZE_MM) { g_self.py=BOX_SIZE_MM; } 
        if(g_self.pz > BOX_SIZE_MM) { g_self.pz=BOX_SIZE_MM; }
        
        if(g_self.vx!=0 || g_self.vy!=0) {
            double h = atan2((double)g_self.vy, (double)g_self.vx) * 180.0/M_PI;
            if(h<0) h+=360.0;
            g_self.yaw_cd = (uint16_t)(h * 100);
        }
        g_self.seq_number++;
        xSemaphoreGive(lock_state);
    }
}

static void task_flocking(void *arg) {
    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
        xSemaphoreTake(lock_state, portMAX_DELAY);
        local_state_t me = g_self;
        xSemaphoreGive(lock_state);
        float cohx=0, cohy=0, cohz=0, alx=0, aly=0, alz=0, sepx=0, sepy=0, sepz=0;
        int cnt = 0;
        xSemaphoreTake(lock_nei, portMAX_DELAY);
        for(int i=0; i<6; i++) {
            if(!g_nei[i].active) continue;
            float dx = g_nei[i].px - me.px; float dy = g_nei[i].py - me.py; float dz = g_nei[i].pz - me.pz;
            float dist = sqrtf(dx*dx + dy*dy + dz*dz);
            cohx+=dx; cohy+=dy; cohz+=dz;
            alx+=(g_nei[i].vx - me.vx); aly+=(g_nei[i].vy - me.vy); alz+=(g_nei[i].vz - me.vz);
            if(dist < SEP_RADIUS_MM && dist > 1.0f) { sepx-=dx/dist; sepy-=dy/dist; sepz-=dz/dist; }
            cnt++;
        }
        xSemaphoreGive(lock_nei);
        if(cnt == 0) continue;
        cohx/=cnt; cohy/=cnt; cohz/=cnt; alx/=cnt; aly/=cnt; alz/=cnt;
// 1) 
float nvx = me.vx + K_COH*cohx + K_ALIGN*alx + K_SEP*sepx;
float nvy = me.vy + K_COH*cohy + K_ALIGN*aly + K_SEP*sepy;
float nvz = me.vz + K_COH*cohz + K_ALIGN*alz + K_SEP*sepz;

// 2) 
float horiz = sqrtf(nvx*nvx + nvy*nvy);

// 2.1 horizontal speed clip
if (horiz > MAX_SPEED_XY_MM_S) {
    float k = MAX_SPEED_XY_MM_S / horiz;
    nvx *= k;
    nvy *= k;
    horiz = MAX_SPEED_XY_MM_S;
}

// 2.2 horizontal speed clip
if (horiz < MIN_SPEED_XY_MM_S && horiz > 1.0f) {
    float k = MIN_SPEED_XY_MM_S / horiz;
    nvx *= k;
    nvy *= k;
}

// 2.3 perpendicular speed clip
if (nvz >  MAX_SPEED_Z_MM_S)  nvz =  MAX_SPEED_Z_MM_S;
if (nvz < -MAX_SPEED_Z_MM_S)  nvz = -MAX_SPEED_Z_MM_S;

// 3) map to global 
xSemaphoreTake(lock_state, portMAX_DELAY);
g_self.vx = (int32_t)nvx;
g_self.vy = (int32_t)nvy;
g_self.vz = (int32_t)nvz;
xSemaphoreGive(lock_state);


    }
}

static void task_lora_tx(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    int log_counter = 0;

    static int64_t last_tx_us = 0;
    static uint32_t tx_count = 0;

    while (1) {
        uint32_t delay_ms = g_attack_flood_enabled ? 200 : 500;

        vTaskDelayUntil(&last, pdMS_TO_TICKS(delay_ms));

        int64_t now_us = esp_timer_get_time();

       if (last_tx_us != 0) {
        int64_t interval_us = now_us - last_tx_us;
        int64_t expected_us = (int64_t)delay_ms * 1000LL;
        int64_t jitter_us   = interval_us - expected_us;

        perf_stats_update(&tx_jitter_stats, jitter_us);

        if (tx_jitter_stats.count % 50 == 0) {
            ESP_LOGI(TAG,
                "[PERF][TX] n=%lld mean=%lld us min=%lld max=%lld",
                tx_jitter_stats.count,
                perf_stats_mean(&tx_jitter_stats),
                tx_jitter_stats.min,
                tx_jitter_stats.max);
        }
    }
 
        
        last_tx_us = now_us;

        xSemaphoreTake(lock_state, portMAX_DELAY);
        local_state_t s = g_self;
        xSemaphoreGive(lock_state);

        comp0221_packet_t f;
        memset(&f, 0, sizeof(f));

        if (g_attack_replay_enabled && g_has_valid_frame) {
            f = g_last_valid_frame;
            f.ts_s += 3600;
            f.ts_ms = (uint16_t)(esp_random() % 1000);
            static uint16_t fake_seq = 50000;
            f.seq_number = fake_seq++;
            ESP_LOGW(TAG, "TX: Replay Attack (Fake TS: %u)", f.ts_s);
        } else {
            f.version = 1;
            f.team_id = TEAM_ID;
            memcpy(f.node_id, s.mac, 6);
            f.seq_number = (uint16_t)(s.seq_number & 0xFFFF);

            struct timeval tv;
            gettimeofday(&tv, NULL);
            f.ts_s  = (uint32_t)tv.tv_sec;
            f.ts_ms = (uint16_t)(tv.tv_usec / 1000);

            f.x_mm = s.px; f.y_mm = s.py; f.z_mm = s.pz;
            f.vx_mm_s = s.vx; f.vy_mm_s = s.vy; f.vz_mm_s = s.vz;
            f.yaw_cd = s.yaw_cd;

            g_last_valid_frame = f;
            g_has_valid_frame = true;

            if (g_attack_flood_enabled)
                ESP_LOGI(TAG, "TX: Flood Packet (5Hz, Seq=%u)", f.seq_number);
            else if (log_counter++ % 20 == 0)
                ESP_LOGI(TAG, "TX: Normal Packet (Seq=%u)", f.seq_number);
        }

        compute_cmac4((uint8_t *)&f, sizeof(f) - 4, f.mac_tag);

        xSemaphoreTake(lock_lora, portMAX_DELAY);
        lora_send_packet((uint8_t *)&f, sizeof(f));
        xSemaphoreGive(lock_lora);

        tx_count++;
    }
}

static void task_lora_rx(void *arg)
{
    uint8_t buf[64];

    static int64_t last_rx_us = 0;
    static uint32_t rx_ok_count = 0;

    while (1) {
        xSemaphoreTake(lock_lora, portMAX_DELAY);
        int n = lora_receive_packet(buf, sizeof(buf));
        xSemaphoreGive(lock_lora);

        vTaskDelay(pdMS_TO_TICKS(20)); // prevent watchdog

        if (n != sizeof(comp0221_packet_t))
            continue;

        int8_t rssi = (int8_t)lora_read_reg(0x1A) - 157;
        int8_t snr  = (int8_t)lora_read_reg(0x19) / 4;

        comp0221_packet_t *f = (comp0221_packet_t *)buf;

        uint8_t tag[4];
        compute_cmac4(buf, sizeof(comp0221_packet_t) - 4, tag);
        if (memcmp(tag, f->mac_tag, 4) != 0)
            continue;

        if (f->team_id != TEAM_ID && f->team_id != 0)
            continue;

        uint8_t mymac[6];
        esp_read_mac(mymac, ESP_MAC_WIFI_STA);
        if (memcmp(mymac, f->node_id, 6) == 0)
            continue;

        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint32_t now_s = (uint32_t)tv.tv_sec;

        if (f->ts_s > now_s + 10 || (now_s > 10 && f->ts_s < now_s - 10)) {
            ESP_LOGW(TAG,
                "RX DROP: Invalid timestamp (pkt=%u, now=%u)",
                f->ts_s, now_s);
            continue;
        }

        /* ---------- PERF: RX INTERVAL / RSSI / SNR ---------- */
        int64_t now_us = esp_timer_get_time();
        if (last_rx_us != 0) {
            int64_t rx_interval_us = now_us - last_rx_us;

            perf_stats_update(&rx_interval_stats, rx_interval_us);
            perf_stats_update(&rssi_stats, rssi);
            perf_stats_update(&snr_stats, snr);

            if (rx_interval_stats.count % 20 == 0) {
                ESP_LOGI(TAG,
                    "[PERF][RX] interval_mean=%lld us RSSI_mean=%lld dBm SNR_mean=%lld dB",
                    perf_stats_mean(&rx_interval_stats),
                    perf_stats_mean(&rssi_stats),
                    perf_stats_mean(&snr_stats));
            }
        }
        last_rx_us = now_us;
        /* ---------------------------------------------------- */

        xSemaphoreTake(lock_nei, portMAX_DELAY);
        int idx = -1;
        for (int i = 0; i < 6; i++)
            if (g_nei[i].active &&
                memcmp(g_nei[i].mac, f->node_id, 6) == 0) {
                idx = i;
                break;
            }

        if (idx < 0)
            for (int i = 0; i < 6; i++)
                if (!g_nei[i].active) {
                    idx = i;
                    break;
                }

        if (idx >= 0) {
            g_nei[idx].active = true;
            memcpy(g_nei[idx].mac, f->node_id, 6);
            g_nei[idx].last_heard = now_ms();
            g_nei[idx].px = f->x_mm;
            g_nei[idx].py = f->y_mm;
            g_nei[idx].pz = f->z_mm;
            g_nei[idx].vx = f->vx_mm_s;
            g_nei[idx].vy = f->vy_mm_s;
            g_nei[idx].vz = f->vz_mm_s;

            ESP_LOGW(TAG,
                ">>> RX PACKET: Node %02X (Seq=%u)",
                f->node_id[5], f->seq_number);
        }
        xSemaphoreGive(lock_nei);

        rx_ok_count++;
    }
}


static void task_nei_expire(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t t = now_ms();
        int active_cnt = 0;

        xSemaphoreTake(lock_nei, portMAX_DELAY);
        for (int i = 0; i < 6; i++) {
            if (g_nei[i].active) {
                if (t - g_nei[i].last_heard > NEI_TIMEOUT_MS) {
                    g_nei[i].active = false;
                } else {
                    active_cnt++;
                }
            }
        }
        xSemaphoreGive(lock_nei);

        // --- Performance log (1 Hz, lightweight) ---
        ESP_LOGI(TAG, "[PERF][NEI] active_neighbours=%d", active_cnt);
    }
}


static void task_telemetry(void *arg) {
    char msg[512]; char str_mac[9];
    struct timeval tv; comp0221_packet_t f;
    int mqtt_log_count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (!mqtt_client) continue;

        xSemaphoreTake(lock_state, portMAX_DELAY);
        local_state_t s = g_self;
        xSemaphoreGive(lock_state);

        memset(&f, 0, sizeof(f));
        f.version = 1; f.team_id = TEAM_ID;
        memcpy(f.node_id, s.mac, 6);
        f.seq_number = (uint16_t)(s.seq_number & 0xFFFF);
        gettimeofday(&tv, NULL);
        f.ts_s = (uint32_t)tv.tv_sec; f.ts_ms = (uint16_t)(tv.tv_usec / 1000);
        f.x_mm = s.px; f.y_mm = s.py; f.z_mm = s.pz;
        f.vx_mm_s = s.vx; f.vy_mm_s = s.vy; f.vz_mm_s = s.vz;
        f.yaw_cd = s.yaw_cd;
        
        compute_cmac4((uint8_t*)&f, sizeof(f)-4, f.mac_tag);
        snprintf(str_mac, sizeof(str_mac), "%02X%02X%02X%02X", f.mac_tag[0], f.mac_tag[1], f.mac_tag[2], f.mac_tag[3]);

        snprintf(msg, sizeof(msg),
            "{\"version\":1,\"team_id\":%d,\"node_id\":\"%02X%02X%02X%02X%02X%02X\","
            "\"seq_number\":%" PRIu32 ",\"ts_s\":%" PRIu32 ",\"ts_ms\":%" PRIu32 ","
            "\"x_mm\":%" PRId32 ",\"y_mm\":%" PRId32 ",\"z_mm\":%" PRId32 ","
            "\"vx_mm_s\":%" PRId32 ",\"vy_mm_s\":%" PRId32 ",\"vz_mm_s\":%" PRId32 ","
            "\"yaw_cd\":%" PRIu16 ",\"mac_tag\":\"%s\"}",
            TEAM_ID,
            s.mac[0], s.mac[1], s.mac[2], s.mac[3], s.mac[4], s.mac[5],
            (uint32_t)f.seq_number, (uint32_t)f.ts_s, (uint32_t)f.ts_ms,
            f.x_mm, f.y_mm, f.z_mm, f.vx_mm_s, f.vy_mm_s, f.vz_mm_s, f.yaw_cd, str_mac
        );
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, msg, 0, 1, 0);
        if (msg_id != -1) {
            if (mqtt_log_count++ % 20 == 0) ESP_LOGI(TAG, "MQTT Sent (Seq: %u)", f.seq_number);
        } else ESP_LOGW(TAG, "MQTT Failed");
    }
}

static void wifi_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) esp_wifi_connect();
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    if ((esp_mqtt_event_id_t)event_id == MQTT_EVENT_CONNECTED) ESP_LOGI(TAG, "MQTT Connected");
    else if ((esp_mqtt_event_id_t)event_id == MQTT_EVENT_DISCONNECTED) ESP_LOGI(TAG, "MQTT Disconnected");
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    perf_stats_init(&tx_jitter_stats);
    perf_stats_init(&rx_interval_stats);
    perf_stats_init(&rssi_stats);
    perf_stats_init(&snr_stats);



    lock_state = xSemaphoreCreateMutex(); lock_nei = xSemaphoreCreateMutex(); lock_lora = xSemaphoreCreateMutex();
    wifi_events = xEventGroupCreate();
    memset(&g_self, 0, sizeof(g_self)); esp_read_mac(g_self.mac, ESP_MAC_WIFI_STA);
    g_self.px = 50000 + (esp_random() % 10000);
    g_self.py = 50000 + (esp_random() % 10000);
    g_self.pz = 50000 + (esp_random() % 10000);

    esp_netif_init(); esp_event_loop_create_default(); esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&cfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_handler, NULL);

    wifi_config_t wc = {0};
#if WIFI_MODE_EDUROAM == 1
    ESP_LOGI(TAG, "WiFi Mode: EDUROAM");
    strcpy((char *)wc.sta.ssid, WIFI_SSID_EDU);
    wc.sta.threshold.authmode = WIFI_AUTH_OPEN;
    esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_eap_client_set_identity((uint8_t*)EAP_ID, strlen(EAP_ID));
    esp_eap_client_set_username((uint8_t*)EAP_USERNAME, strlen(EAP_USERNAME));
    esp_eap_client_set_password((uint8_t*)EAP_PASSWORD, strlen(EAP_PASSWORD));
    esp_wifi_sta_enterprise_enable();
#else
    ESP_LOGI(TAG, "WiFi Mode: HOME");
    strcpy((char *)wc.sta.ssid, HOME_WIFI_SSID);
    strcpy((char *)wc.sta.password, HOME_WIFI_PASSWORD);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_set_config(WIFI_IF_STA, &wc);
#endif
    esp_wifi_start();
    xEventGroupWaitBits(wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    init_sntp();
    esp_mqtt_client_config_t mcfg = { .broker.address.uri = MQTT_URI };
    mqtt_client = esp_mqtt_client_init(&mcfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    lora_init();
    lora_set_frequency((long)(LORA_FREQ * 1000000)); 
    lora_enable_crc();

    xTaskCreate(task_physics,    "physics",   4096, NULL, 4, NULL);
    xTaskCreate(task_flocking,   "flocking",  4096, NULL, 3, NULL);
    xTaskCreate(task_lora_tx,    "lora_tx",   4096, NULL, 3, NULL);
    xTaskCreate(task_lora_rx,    "lora_rx",   4096, NULL, 3, NULL);
    xTaskCreate(task_nei_expire, "nei_exp",   2048, NULL, 2, NULL);
    xTaskCreate(task_telemetry,  "telemetry", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "System initialised (CONFIGURABLE MODE).");
}