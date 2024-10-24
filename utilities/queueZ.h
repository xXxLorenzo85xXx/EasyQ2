#include <zephyr/zephyr.h>
#include <zephyr/kernel.h>

typedef struct {
    struct k_msgq msgq;
    uint32_t fifo_capacity; // How many items can be stored
    uint32_t num_item; // number of data item
    uint32_t item_size; // data size
    uint32_t ovf_item; // Number of overflowed data
    uint32_t buffer_size; // buffer size in bytes
    uint32_t max_buffer_size; // buffer size in bytes
    char *name;
#ifdef QUEUE_USAGE_STATS
    uint32_t pop_cnt;
    uint32_t push_cnt;
    uint32_t stats_period_cnt;
#endif
} queue_t;