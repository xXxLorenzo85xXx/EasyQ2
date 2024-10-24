#include <zephyr/zephyr.h>
#include <zephyr/kernel.h>
#include "queueZ.h"

int queue_init(queue_t *q, void *buf, uint32_t item_size, uint32_t msg_count)
{
    if (!q || !buf || !msg_count || !item_size)
        return -EINVAL;

    k_msgq_init(&q->msgq,&buf, sizeof(queue_t), msg_count);
    q->msgq.buffer_start = buf;
    q->msgq.msg_size = item_size;
    q->msgq.max_msgs = msg_count;
    q->item_size = item_size;
    q->buffer_size = item_size * msg_count;
    q->max_buffer_size = q->buffer_size;

    return 0;
}

int enqueue(queue_t *q, void *data) {
    if (!q || !data)
        return -EINVAL;

    return k_msgq_put(&q->msgq, data, K_NO_WAIT);
}

int dequeue(queue_t *q, void *data) {
    if (!q || !data)
        return -EINVAL;

    return k_msgq_get(&q->msgq, data, K_NO_WAIT);
}

int queue_len(queue_t *q) {
    if (!q)
        return -EINVAL;

    return k_msgq_num_used_get(&q->msgq);
}

int queue_capacity(queue_t *q) {
    if (!q)
        return -EINVAL;

    return k_msgq_num_free_get(&q->msgq);
}

