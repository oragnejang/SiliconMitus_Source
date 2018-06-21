/*
*	USB PD Driver - Protocol Layer
*/

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ccic/usbpd.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/ccic/usbpd-sm5713.h>
#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
#include <linux/battery/battery_notifier.h>
#endif

#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
extern struct pdic_notifier_struct pd_noti;
#endif

static void increase_message_id_counter(struct usbpd_data *pd_data)
{
	pd_data->counter.message_id_counter++;
	pd_data->counter.message_id_counter %= 8;
/*
	if (pd_data->counter.message_id_counter++ > USBPD_nMessageIDCount)
		pd_data->counter.message_id_counter = 0;
*/
}

static void rx_layer_init(struct protocol_data *rx)
{
	int i;

	rx->stored_message_id = USBPD_nMessageIDCount+1;
	rx->msg_header.word = 0;
	rx->state = 0;
	rx->status = DEFAULT_PROTOCOL_NONE;
	for (i = 0; i < USBPD_MAX_COUNT_MSG_OBJECT; i++)
		rx->data_obj[i].object = 0;
}

static void tx_layer_init(struct protocol_data *tx)
{
	int i;

	tx->stored_message_id = USBPD_nMessageIDCount+1;
	tx->msg_header.word = 0;
	tx->state = 0;
	tx->status = DEFAULT_PROTOCOL_NONE;
	for (i = 0; i < USBPD_MAX_COUNT_MSG_OBJECT; i++)
		tx->data_obj[i].object = 0;
}

static void tx_discard_message(struct protocol_data *tx)
{
	int i;

	tx->msg_header.word = 0;
	for (i = 0; i < USBPD_MAX_COUNT_MSG_OBJECT; i++)
		tx->data_obj[i].object = 0;
}

void usbpd_tx_request_discard(struct usbpd_data *pd_data)
{
    struct protocol_data *tx = &pd_data->protocol_tx;
    tx_discard_message(tx);
    dev_err(pd_data->dev, "%s\n", __func__);
}

void usbpd_init_protocol(struct usbpd_data *pd_data)
{
	struct policy_data *policy = &pd_data->policy;
    if (policy->state == PE_SRC_Startup || policy->state == PE_SNK_Startup || policy->state == PE_SRC_Send_Soft_Reset || policy->state == PE_SNK_Soft_Reset) {
        sm5713_protocol_layer_reset(pd_data);
    }
	rx_layer_init(&pd_data->protocol_rx);
	tx_layer_init(&pd_data->protocol_tx);
}

void usbpd_init_counters(struct usbpd_data *pd_data)
{
	pr_info("%s: init counter\n", __func__);
	pd_data->counter.retry_counter = 0;
	pd_data->counter.message_id_counter = 0;
	pd_data->counter.caps_counter = 0;
	pd_data->counter.hard_reset_counter = 0;
	pd_data->counter.swap_hard_reset_counter = 0;
	pd_data->counter.discover_identity_counter = 0;
}

void usbpd_policy_reset(struct usbpd_data *pd_data, unsigned flag)
{
	if (flag == HARDRESET_RECEIVED) {
		pd_data->policy.rx_hardreset = 1;
		dev_info(pd_data->dev, "%s Hard reset\n", __func__);
	} else if (flag == SOFTRESET_RECEIVED) {
		pd_data->policy.rx_softreset = 1;
		dev_info(pd_data->dev, "%s Soft reset\n", __func__);
	} else if (flag == PLUG_EVENT) {
		pd_data->policy.plug = 1;
		pd_data->policy.plug_valid = 1;
		dev_info(pd_data->dev, "%s ATTACHED\n", __func__);
	} else if (flag == PLUG_DETACHED) {
		pd_data->policy.plug_valid = 0;
		dev_info(pd_data->dev, "%s DETACHED\n", __func__);
	}
}

protocol_state usbpd_protocol_tx_phy_layer_reset(struct protocol_data *tx)
{
	return PRL_Tx_Wait_for_Message_Request;
}

protocol_state usbpd_protocol_tx_wait_for_message_request(struct protocol_data
								*tx)
{
	struct usbpd_data *pd_data = protocol_tx_to_usbpd(tx);
	protocol_state state = PRL_Tx_Wait_for_Message_Request;

	/* S2MM004 PDIC already retry.
	if (pd_data->counter.retry_counter > USBPD_nRetryCount) {
		pd_data->counter.retry_counter = 0;
		return state;
	}
	*/
	if (pd_data->counter.retry_counter > 0) {
		pd_data->counter.retry_counter = 0;
		return state;
	}

	pd_data->counter.retry_counter = 0;

	if (!tx->msg_header.word)
		return state;

	if (tx->msg_header.num_data_objs == 0 &&
			tx->msg_header.msg_type == USBPD_Soft_Reset)
		state = PRL_Tx_Layer_Reset_for_Transmit;
	else
		state = PRL_Tx_Construct_Message;

	return state;
}

protocol_state usbpd_protocol_tx_layer_reset_for_transmit(struct protocol_data *tx)
{
	struct usbpd_data *pd_data = protocol_tx_to_usbpd(tx);

	dev_info(pd_data->dev, "%s\n", __func__);

	pd_data->counter.message_id_counter = 0;
	pd_data->protocol_rx.state = PRL_Rx_Wait_for_PHY_Message;

	/* TODO: check Layer Reset Complete */
	return PRL_Tx_Construct_Message;
}

protocol_state usbpd_protocol_tx_construct_message(struct protocol_data *tx)
{
	struct usbpd_data *pd_data = protocol_tx_to_usbpd(tx);

	tx->msg_header.msg_id = pd_data->counter.message_id_counter;
	tx->status = DEFAULT_PROTOCOL_NONE;

	if (pd_data->phy_ops.tx_msg(pd_data, &tx->msg_header, tx->data_obj)) {
		dev_err(pd_data->dev, "%s error\n", __func__);
		return PRL_Tx_Construct_Message;
	}
	return PRL_Tx_Wait_for_PHY_Response;
}

protocol_state usbpd_protocol_tx_wait_for_phy_response(struct protocol_data *tx)
{
#if 0
	struct usbpd_data *pd_data = protocol_tx_to_usbpd(tx);
	protocol_state state = PRL_Tx_Wait_for_PHY_Response;
	u8 CrcCheck_cnt = 0;

	/* wait to get goodcrc */
	/* mdelay(1); */

	/* polling */
	/* pd_data->phy_ops.poll_status(pd_data); */

	for (CrcCheck_cnt = 0; CrcCheck_cnt < 2; CrcCheck_cnt++) {
		if (pd_data->phy_ops.get_status(pd_data, MSG_GOODCRC)) {
			pr_info("%s : %p\n", __func__, pd_data);
			state = PRL_Tx_Message_Sent;
			dev_info(pd_data->dev, "got GoodCRC.\n");
			return state;
		}

		if (!CrcCheck_cnt)
			pd_data->phy_ops.poll_status(pd_data); /* polling */
	}

	return PRL_Tx_Check_RetryCounter;
#endif
	return PRL_Tx_Message_Sent;
}

protocol_state usbpd_protocol_tx_match_messageid(struct protocol_data *tx)
{
	/* We don't use this function.
	   S2MM004 PDIC already check message id for incoming GoodCRC.

	struct usbpd_data *pd_data = protocol_tx_to_usbpd(protocol_tx);
	protocol_state state = PRL_Tx_Match_MessageID;

	dev_info(pd_data->dev, "%s\n",__func__);

	if (pd_data->protocol_rx.msg_header.msg_id
			== pd_data->counter.message_id_counter)
		state = PRL_Tx_Message_Sent;
	else
		state = PRL_Tx_Check_RetryCounter;

	return state;
	*/
	return PRL_Tx_Message_Sent;
}

protocol_state usbpd_protocol_tx_message_sent(struct protocol_data *tx)
{
	struct usbpd_data *pd_data = protocol_tx_to_usbpd(tx);

	increase_message_id_counter(pd_data);
	tx->status = MESSAGE_SENT;
	/* clear protocol header buffer */
	tx->msg_header.word = 0;

	return PRL_Tx_Wait_for_Message_Request;
}

protocol_state usbpd_protocol_tx_check_retrycounter(struct protocol_data *tx)
{
	struct usbpd_data *pd_data = protocol_tx_to_usbpd(tx);

	/* S2MM004 PDIC already do retry.
	   Driver SW doesn't do retry.

	if (++pd_data->counter.retry_counter > USBPD_nRetryCount) {
		state = PRL_Tx_Transmission_Error;
	} else {
		state = PRL_Tx_Construct_Message;
	}

	return PRL_Tx_Check_RetryCounter;
	*/
	++pd_data->counter.retry_counter;
	return PRL_Tx_Transmission_Error;
}

protocol_state usbpd_protocol_tx_transmission_error(struct protocol_data *tx)
{
	struct usbpd_data *pd_data = protocol_tx_to_usbpd(tx);

	dev_err(pd_data->dev, "%s\n", __func__);

	increase_message_id_counter(pd_data);
	tx->status = TRANSMISSION_ERROR;

	return PRL_Tx_Wait_for_Message_Request;
}

protocol_state usbpd_protocol_tx_discard_message(struct protocol_data *tx)
{
	/* This state is for Only Ping message */
	struct usbpd_data *pd_data = protocol_tx_to_usbpd(tx);

	dev_err(pd_data->dev, "%s\n", __func__);
	tx_discard_message(tx);
	increase_message_id_counter(pd_data);

	return PRL_Tx_PHY_Layer_Reset;
}

void usbpd_set_ops(struct device *dev, usbpd_phy_ops_type *ops)
{
	struct usbpd_data *pd_data = (struct usbpd_data *) dev_get_drvdata(dev);

	pd_data->phy_ops.tx_msg = ops->tx_msg;
	pd_data->phy_ops.rx_msg = ops->rx_msg;
	pd_data->phy_ops.hard_reset = ops->hard_reset;
	pd_data->phy_ops.set_power_role = ops->set_power_role;
	pd_data->phy_ops.get_power_role = ops->get_power_role;
	pd_data->phy_ops.set_data_role = ops->set_data_role;
	pd_data->phy_ops.get_data_role = ops->get_data_role;
	pd_data->phy_ops.get_vconn_source = ops->get_vconn_source;
	pd_data->phy_ops.set_vconn_source = ops->set_vconn_source;
	pd_data->phy_ops.set_check_msg_pass = ops->set_check_msg_pass;
	pd_data->phy_ops.get_status = ops->get_status;
	pd_data->phy_ops.poll_status = ops->poll_status;
	pd_data->phy_ops.driver_reset = ops->driver_reset;
}

protocol_state usbpd_protocol_rx_layer_reset_for_receive(struct protocol_data *rx)
{
	struct usbpd_data *pd_data = protocol_rx_to_usbpd(rx);

	dev_info(pd_data->dev, "%s\n", __func__);
	/*
	rx_layer_init(protocol_rx);
	pd_data->protocol_tx.state = PRL_Tx_PHY_Layer_Reset;
	*/

	usbpd_rx_soft_reset(pd_data);
	return PRL_Rx_Layer_Reset_for_Receive;

	/*return PRL_Rx_Send_GoodCRC;*/
}

protocol_state usbpd_protocol_rx_wait_for_phy_message(struct protocol_data *rx)
{
	struct usbpd_data *pd_data = protocol_rx_to_usbpd(rx);
	protocol_state state = PRL_Rx_Wait_for_PHY_Message;

	if (pd_data->phy_ops.rx_msg(pd_data, &rx->msg_header, rx->data_obj)) {
		dev_err(pd_data->dev, "%s IO Error\n", __func__);
		return state;
	} else {
		if (rx->msg_header.word == 0) {
			return state; /* no message */
		} else if (pd_data->phy_ops.get_status(pd_data, MSG_SOFTRESET))	{
			dev_err(pd_data->dev, "[Rx] Got SOFTRESET.\n");
			state = PRL_Rx_Layer_Reset_for_Receive;
		} else {
			if (rx->stored_message_id == rx->msg_header.msg_id)
				return state;

			dev_err(pd_data->dev, "[Rx] [0x%x] [0x%x]\n",
					rx->msg_header.word, rx->data_obj[0].object);
			/* new message is coming */
			state = PRL_Rx_Send_GoodCRC;
		}
	}
	return state;
}

protocol_state usbpd_protocol_rx_send_goodcrc(struct protocol_data *rx)
{
	/* Goodcrc sent by PDIC(HW) */
	return PRL_Rx_Check_MessageID;
}

protocol_state usbpd_protocol_rx_store_messageid(struct protocol_data *rx)
{
	struct usbpd_data *pd_data = protocol_rx_to_usbpd(rx);

	rx->stored_message_id = rx->msg_header.msg_id;
	usbpd_read_msg(pd_data);
/*
	return PRL_Rx_Wait_for_PHY_Message;
*/
	return PRL_Rx_Store_MessageID;
}

protocol_state usbpd_protocol_rx_check_messageid(struct protocol_data *rx)
{
	protocol_state state;

	if (rx->stored_message_id == rx->msg_header.msg_id)
		state = PRL_Rx_Wait_for_PHY_Message;
	else
		state = PRL_Rx_Store_MessageID;
	return state;
}

void usbpd_protocol_tx(struct usbpd_data *pd_data)
{
	struct protocol_data *tx = &pd_data->protocol_tx;

	if (tx->msg_header.num_data_objs == 0 &&
			tx->msg_header.msg_type == USBPD_Soft_Reset) {
        pd_data->counter.message_id_counter = 0; // SM5713 auto increase count.. not need this value
        pd_data->protocol_rx.state = PRL_Rx_Wait_for_PHY_Message;
    }

	tx->msg_header.msg_id = pd_data->counter.message_id_counter;
	tx->status = DEFAULT_PROTOCOL_NONE;

    if (pd_data->phy_ops.tx_msg(pd_data, &tx->msg_header, tx->data_obj)) {
        dev_err(pd_data->dev, "%s error\n", __func__);
    }
    tx->state = PRL_Tx_Message_Sent;
	increase_message_id_counter(pd_data); // SM5713 auto increase count.. not need this function
	/* clear protocol header buffer */
	tx->msg_header.word = 0;    
}

void usbpd_check_vdm(struct usbpd_data *pd_data)
{
    struct sm5713_usbpd_data *pdic_data = pd_data->phy_driver_data;
    unsigned cmd, cmd_type, vdm_type;
    int i;

    cmd = pd_data->policy.rx_data_obj[0].structured_vdm.command;
    cmd_type = pd_data->policy.rx_data_obj[0].structured_vdm.command_type;
    vdm_type = pd_data->policy.rx_data_obj[0].structured_vdm.vdm_type;
    
    dev_info(pd_data->dev, "%s: cmd = %d, cmd_type = %d, vdm_type = %d\n", __func__, cmd, cmd_type, vdm_type);

    if (vdm_type == Unstructured_VDM) {
        for(i = 0; i < pd_data->policy.rx_msg_header.num_data_objs; i++) {
            dev_info(pd_data->dev, "%s: Unstructured_VDM Object[%d] - [0x%x]\n", __func__, i, pd_data->policy.rx_data_obj[i].object);
        }
        return;
    }

    if (cmd_type == Initiator) {
        switch (cmd) {
            case Discover_Identity:
                pdic_data->status_reg |= VDM_DISCOVER_IDENTITY;
                break;
            case Discover_SVIDs:
                pdic_data->status_reg |= VDM_DISCOVER_SVID;
                break;
            case Discover_Modes:
                pdic_data->status_reg |= VDM_DISCOVER_MODE;
                break;
            case Enter_Mode:
                pdic_data->status_reg |= VDM_ENTER_MODE;
                break;
            case Exit_Mode:
                pdic_data->status_reg |= VDM_EXIT_MODE;
                break;
            case Attention:
//                pdic_data->status_reg |= VDM_ATTENTION;
                break;                
            case DisplayPort_Status_Update:
                pdic_data->status_reg |= MSG_PASS;
                break;
            case DisplayPort_Configure:
                pdic_data->status_reg |= MSG_PASS;
                break; 
        }
    } else if (cmd_type == Responder_ACK) {
        switch (cmd) {    
            case Discover_Identity:
                pdic_data->status_reg |= VDM_DISCOVER_IDENTITY;
                break;
            case Discover_SVIDs:
                pdic_data->status_reg |= VDM_DISCOVER_SVID;
                break;
            case Discover_Modes:
                pdic_data->status_reg |= VDM_DISCOVER_MODE;
                break;
            case Enter_Mode:
                pdic_data->status_reg |= VDM_ENTER_MODE;
                break;
            case Exit_Mode:
                pdic_data->status_reg |= VDM_EXIT_MODE;
                break;
#if 0
            case DisplayPort_Status_Update:
                pdic_data->status_reg |= MSG_PASS;
                break;
            case DisplayPort_Configure:
                pdic_data->status_reg |= MSG_PASS;
                break;
#endif
        }                
    } else if (cmd_type == Responder_NAK) {
    } else {// Responder_BUSY
    }
}


void usbpd_protocol_rx(struct usbpd_data *pd_data)
{
	struct protocol_data *rx = &pd_data->protocol_rx;
    struct sm5713_usbpd_data *pdic_data = pd_data->phy_driver_data;
    u8 ext_msg;

	if (pd_data->phy_ops.rx_msg(pd_data, &rx->msg_header, rx->data_obj)) {
		dev_err(pd_data->dev, "%s IO Error\n", __func__);
		return;
	} else {
	    dev_info(pd_data->dev, "%s: stored_message_id = %x, msg_id = %d\n", __func__, rx->stored_message_id, rx->msg_header.msg_id);
		if (rx->msg_header.word == 0) {
            dev_err(pd_data->dev, "[Rx] No Message.\n");
			return; /* no message */
		} else {
			dev_err(pd_data->dev, "[Rx] [0x%x] [0x%x]\n",
					rx->msg_header.word, rx->data_obj[0].object);
		}
	}

//	if (rx->stored_message_id != rx->msg_header.msg_id) {
        rx->stored_message_id = rx->msg_header.msg_id;

        usbpd_read_msg(pd_data);

        ext_msg = pd_data->policy.rx_msg_header.byte[1] & 0x80;
        dev_info(pd_data->dev, "%s: ext_msg = %x, obj_num = %d, msg_type = %d\n", 
            __func__, ext_msg, pd_data->policy.rx_msg_header.num_data_objs, pd_data->policy.rx_msg_header.msg_type);

        if (ext_msg && pd_data->policy.rx_msg_header.spec_revision == 0) {
            return;
        } else if (pd_data->policy.rx_msg_header.num_data_objs > 0) {
            switch(pd_data->policy.rx_msg_header.msg_type) {
                case USBPD_Source_Capabilities:
                    pdic_data->status_reg |= MSG_SRC_CAP;
                    break;
                case USBPD_Request:
                    pdic_data->status_reg |= MSG_REQUEST;
                    break;
                case USBPD_BIST:
                    if (pd_data->policy.state == PE_SNK_Ready || pd_data->policy.state == PE_SRC_Ready) {
                        // ST_PE_BIST_CARRIER_M2
                    }                    
                    break;
                case USBPD_Sink_Capabilities:
                    pdic_data->status_reg |= MSG_SNK_CAP;
                    break;
                case USBPD_Vendor_Defined:
                    // handle_vdm_request()
                    usbpd_check_vdm(pd_data);
                    break;
                default:
                    break;
            }
        } else {
            switch(pd_data->policy.rx_msg_header.msg_type) {
                case USBPD_GoodCRC:
                    /* Do nothing */
                    break;
                case USBPD_Ping:
                    /* Do nothing */
                    break;
                case USBPD_GotoMin:
                    if (pd_data->policy.state == PE_SNK_Ready) {
                        pd_data->policy.state = PE_SNK_Transition_Sink;
                    }
                    break;
                case USBPD_Accept:
                    pdic_data->status_reg |= MSG_ACCEPT;
                    break;                    
                case USBPD_Reject:
                    pdic_data->status_reg |= MSG_REJECT;
                    break;
                case USBPD_PS_RDY:
                    pdic_data->status_reg |= MSG_PSRDY;                    
                    break;
                case USBPD_Get_Source_Cap:
                    pdic_data->status_reg |= MSG_GET_SRC_CAP;
                    break;
                case USBPD_Get_Sink_Cap:
                    pdic_data->status_reg |= MSG_GET_SNK_CAP;
                    break;
                case USBPD_DR_Swap:
                    if (pd_data->policy.state == PE_SNK_Ready || pd_data->policy.state == PE_SRC_Ready) {
                        pdic_data->status_reg |= MSG_DR_SWAP;
                    }                    
                    break;
                case USBPD_PR_Swap:
                    if (pd_data->policy.state == PE_SNK_Ready || pd_data->policy.state == PE_SRC_Ready) {
                        pdic_data->status_reg |= MSG_PR_SWAP;
                    }
                    break;
                case USBPD_VCONN_Swap:
                    if (pd_data->policy.state == PE_SNK_Ready || pd_data->policy.state == PE_SRC_Ready) {
                        pdic_data->status_reg |= MSG_VCONN_SWAP;
                    }                    
                    break;
                case USBPD_Wait:
/*                    if (pd_data->policy.state == PE_SNK_Select_Capability) {
                        pdic_data->status_reg |= MSG_WAIT;
                    } else if (pd_data->policy.state == PE_DRS_DFP_UFP_Send_DR_Swap) {
                        pdic_data->status_reg |= MSG_WAIT;
                    } else if (pd_data->policy.state == PE_DRS_UFP_DFP_Send_DR_Swap) {
                        pdic_data->status_reg |= MSG_WAIT;
                    }*/
                    pdic_data->status_reg |= MSG_WAIT;
                    break;
                case USBPD_Soft_Reset:
                    pdic_data->status_reg |= MSG_SOFTRESET;
                    break;
                default:
                    break;
            }
        }
//    }
}

void usbpd_read_msg(struct usbpd_data *pd_data)
{
	int i;

	pd_data->policy.rx_msg_header.word
		= pd_data->protocol_rx.msg_header.word;
	for (i = 0; i < USBPD_MAX_COUNT_MSG_OBJECT; i++) {
		pd_data->policy.rx_data_obj[i].object
			= pd_data->protocol_rx.data_obj[i].object;
	}
}

/* return 1: sent with goodcrc, 0: fail */
bool usbpd_send_msg(struct usbpd_data *pd_data, msg_header_type *header,
		data_obj_type *obj)
{
	int i;

	if (obj)
		for (i = 0; i < USBPD_MAX_COUNT_MSG_OBJECT; i++)
			pd_data->protocol_tx.data_obj[i].object = obj[i].object;
	else
		header->num_data_objs = 0;

	header->spec_revision = USBPD_REV_20;
	pd_data->protocol_tx.msg_header.word = header->word;
	usbpd_protocol_tx(pd_data);

    return true;
}

inline bool usbpd_send_ctrl_msg(struct usbpd_data *d, msg_header_type *h,
		unsigned msg, unsigned dr, unsigned pr)
{
	h->msg_type = msg;
	h->port_data_role = dr;
	h->port_power_role = pr;
	h->num_data_objs = 0;
	return usbpd_send_msg(d, h, 0);
}

/* return: 0 if timed out, positive is status */
inline unsigned usbpd_wait_msg(struct usbpd_data *pd_data,
				unsigned msg_status, unsigned ms)
{
	unsigned long ret;

	ret = pd_data->phy_ops.get_status(pd_data, msg_status);
	if (ret) {
		pd_data->policy.abnormal_state = false;
		return ret;
	}
    dev_info(pd_data->dev, "%s: msg_status = %d, time = %d\n", __func__, msg_status, ms);
	/* wait */
	reinit_completion(&pd_data->msg_arrived);
	pd_data->wait_for_msg_arrived = msg_status;
	ret = wait_for_completion_timeout(&pd_data->msg_arrived,
			msecs_to_jiffies(ms));

	if (!pd_data->policy.state) {
		dev_err(pd_data->dev, "%s : return for policy state error\n", __func__);
		pd_data->policy.abnormal_state = true;
		return 0;
	}

	pd_data->policy.abnormal_state = false;

	return pd_data->phy_ops.get_status(pd_data, msg_status);
}

void usbpd_rx_hard_reset(struct device *dev)
{
	struct usbpd_data *pd_data = dev_get_drvdata(dev);

	usbpd_reinit(dev);
	usbpd_policy_reset(pd_data, HARDRESET_RECEIVED);
}

void usbpd_rx_soft_reset(struct usbpd_data *pd_data)
{
	usbpd_reinit(pd_data->dev);
	usbpd_policy_reset(pd_data, SOFTRESET_RECEIVED);
}

void usbpd_reinit(struct device *dev)
{
	struct usbpd_data *pd_data = dev_get_drvdata(dev);

	usbpd_init_counters(pd_data);
	usbpd_init_protocol(pd_data);
	usbpd_init_policy(pd_data);
	reinit_completion(&pd_data->msg_arrived);
	pd_data->wait_for_msg_arrived = 0;
	complete(&pd_data->msg_arrived);
}

/*
 * usbpd_init - alloc usbpd data
 *
 * Returns 0 on success; negative errno on failure
*/
int usbpd_init(struct device *dev, void *phy_driver_data)
{
	struct usbpd_data *pd_data;

	if (!dev)
		return -EINVAL;

	pd_data = kzalloc(sizeof(struct usbpd_data), GFP_KERNEL);

	if (!pd_data)
		return -ENOMEM;

	pd_data->dev = dev;
	pd_data->phy_driver_data = phy_driver_data;
	dev_set_drvdata(dev, pd_data);

#ifdef CONFIG_USB_TYPEC_MANAGER_NOTIFIER
	pd_noti.pd_data = pd_data;
	pd_noti.sink_status.current_pdo_num = 0;
	pd_noti.sink_status.selected_pdo_num = 0;
#endif
	usbpd_init_counters(pd_data);
	usbpd_init_protocol(pd_data);
	usbpd_init_policy(pd_data);
	usbpd_init_manager(pd_data);

	INIT_WORK(&pd_data->worker, usbpd_policy_work);
	init_completion(&pd_data->msg_arrived);

	return 0;
}
