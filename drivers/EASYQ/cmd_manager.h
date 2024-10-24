#include <zephyr/kernel.h>

/* MAX ENTRIES*/
#define CMD_EVENT_QUEUE_SIZE 20

enum cmd_type
{
    CMD_TYPE_INIT_PMU = 0,
    CMD_TYPE_RUN_PMU,
    CMD_TYPE_INIT_ACC,
    CMD_TYPE_RUN_ACC,
    CMD_TYPE_INIT_MAX86178,
    CMD_TYPE_RUN_MAX86178,
    CMD_TYPE_RUN_ECG,
    CMD_TYPE_INIT_FLASH,
    CMD_TYPE_RUN_FLASH,
    CMD_TYPE_STOP,
};

enum cmd_type_secondary
{
    CMD2_TYPE_INIT_PMU = 0,
    CMD2_TYPE_RUN_PMU,
    CMD2_TYPE_INIT_ACC,
    CMD2_TYPE_RUN_ACC,
    CMD2_TYPE_INIT_MAX86178,
    CMD2_TYPE_RUN_MAX86178,
    CMD2_TYPE_RUN_ECG,
    CMD2_TYPE_INIT_FLASH,
    CMD2_TYPE_RUN_FLASH,
};

struct cmd_item
{
    enum cmd_type cmd;
    enum cmd_type_secondary cmd2;
    u_int8_t register_addr;
    u_int8_t value;
    u_int8_t err;
    u_int8_t size;
};



int cmd_event_manager_put(struct cmd_item *cmd_evt, struct k_msgq *msgq);

int cmd_event_manager_get(struct cmd_item *cmd_evt, struct k_msgq *msgq);