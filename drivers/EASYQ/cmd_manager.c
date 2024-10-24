#include <zephyr/kernel.h>
#include "cmd_manager.h"

int cmd_event_manager_put(struct cmd_item *cmd_evt, struct k_msgq *msgq)
{
        return k_msgq_put(msgq, cmd_evt, K_NO_WAIT);
}

int cmd_event_manager_get(struct cmd_item *cmd_evt, struct k_msgq *msgq)
{
        return k_msgq_get(msgq, cmd_evt, K_FOREVER);
}