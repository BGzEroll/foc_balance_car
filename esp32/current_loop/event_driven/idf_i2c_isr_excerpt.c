    portBASE_TYPE HPTaskAwoken = pdFALSE;
    uint32_t int_mask;
    BaseType_t ret = pdTRUE;
    i2c_master->trans_done = false;
    i2c_ll_get_intr_mask(hal->dev, &int_mask);
    i2c_ll_clear_intr_mask(hal->dev, int_mask);
    if (int_mask == 0) {
        return;
    }

    if (int_mask & I2C_LL_INTR_NACK) {
        atomic_store(&i2c_master->status, I2C_STATUS_ACK_ERROR);
        i2c_master->event = I2C_EVENT_NACK;
    } else if (int_mask & I2C_LL_INTR_TIMEOUT || int_mask & I2C_LL_INTR_ARBITRATION) {
        atomic_store(&i2c_master->status, I2C_STATUS_TIMEOUT);
        i2c_master->event = I2C_EVENT_TIMEOUT;
    } else if (int_mask & I2C_LL_INTR_MST_COMPLETE) {
        i2c_master->trans_done = true;
        i2c_master->event = I2C_EVENT_DONE;
    }
    if (i2c_master->event != I2C_EVENT_ALIVE) {
        xQueueSendFromISR(i2c_master->event_queue, (void *)&i2c_master->event, &HPTaskAwoken);
    }
    if (i2c_master->contains_read == true) {
        if (int_mask & I2C_LL_INTR_MST_COMPLETE || int_mask & I2C_LL_INTR_END_DETECT) {
            i2c_isr_receive_handler(i2c_master);
        }
    }

    if (i2c_master->async_trans) {
        i2c_master_dev_handle_t i2c_dev = NULL;

        i2c_master_device_list_t *device_item;
        xSemaphoreTakeFromISR(i2c_master->bus_lock_mux, &HPTaskAwoken);
        SLIST_FOREACH(device_item, &i2c_master->device_list, next) {
            if (device_item->device->device_address == i2c_master->i2c_trans.device_address) {
                i2c_dev = device_item->device;
                break;
            }
        }
        xSemaphoreGiveFromISR(i2c_master->bus_lock_mux, &HPTaskAwoken);
        if (i2c_dev == NULL) {
            // HPTaskAwoken may have been set by xQueueSendFromISR / xSemaphoreTakeFromISR /
            // xSemaphoreGiveFromISR above; fall through to the yield check at the end of
            // the ISR rather than returning directly.
            goto out;
        }
        i2c_master_event_data_t evt = {
            .event = i2c_master->event,
        };
        s_i2c_send_command_async(i2c_master, &HPTaskAwoken);
        if (i2c_master->trans_done) {
            if (i2c_dev->on_trans_done) {
                i2c_dev->on_trans_done(i2c_dev, &evt, i2c_dev->user_ctx);
            }
            if (i2c_master->new_queue && i2c_master->num_trans_inqueue > 0 && i2c_master->in_progress == false) {
                i2c_transaction_t t = {};
                i2c_master->num_trans_inqueue--;
                i2c_master->new_queue = false;
                t.cmd_count = 0;
                t.device_address = 0;
                t.ops = NULL;
                ret = xQueueReceiveFromISR(i2c_master->trans_queues[I2C_TRANS_QUEUE_PROGRESS], &t, &HPTaskAwoken);
                if (ret == pdTRUE) {
                    i2c_master->queue_trans = true;
                    atomic_init(&i2c_master->trans_idx, 0);
                    atomic_store(&i2c_master->status, I2C_STATUS_IDLE);
                    i2c_master->cmd_idx = 0;
                    i2c_master->rx_cnt = 0;
                    i2c_master->read_len_static = 0;
                    i2c_ll_txfifo_rst(hal->dev);
                    i2c_ll_rxfifo_rst(hal->dev);
                    i2c_master->i2c_trans = t;
                    memcpy(i2c_master->i2c_ops, t.ops, t.cmd_count * sizeof(i2c_operation_t));

                    s_i2c_send_command_async(i2c_master, &HPTaskAwoken);
                }
            }
        }
    } else {
        xSemaphoreGiveFromISR(i2c_master->cmd_semphr, &HPTaskAwoken);
    }

out:
    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }

}
