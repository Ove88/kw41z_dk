/*
*
*/


#include "clang_headers.h" // For completion

#include "ns_types.h"
#include "arm_hal_interrupt.h"
#include "arm_hal_phy.h"

#include "NanostackRfPhyKw41z.h"

#include "PhyInterface.h"
#include "Phy.h"

#define RADIO_PAN_REGISTER    0

/* RF driver data */
static int8_t rf_radio_driver_id = -1;
static uint8_t mac_address[8];
static uint16_t shortMac_address;

static phy_device_driver_s device_driver;

/* Driver instance handle */
static NanostackRfPhyKw41z *rf = NULL;

const phy_rf_channel_configuration_s phy_2_4ghz = {2405000000, 5000000, 250000, 16, M_OQPSK};
const phy_rf_channel_configuration_s phy_subghz = {868300000, 2000000, 250000, 11, M_OQPSK};

static phy_device_channel_page_s phy_channel_pages[] = {
    {CHANNEL_PAGE_0, &phy_2_4ghz},
    {CHANNEL_PAGE_0, NULL}
};


typedef enum {
    STATE_SLEEP,
    STATE_IDLE,
    STATE_TX,
    STATE_RX,
    STATE_CALIBRATION
} radio_state_t;

typedef enum {
    plmeEdCnf_t recvCnf;
    bool        newCnf;
}edConf_t;

static uint8_t ack_requested;
static uint8_t current_tx_handle;
static int8_t current_channel = -1;

static radio_state_t radio_state = RADIO_IDLE;
static const instanceId_t mac_instance_id = 0, phy_instance_id = 1;
static edConf_t lastEdCnf;

/* ARM_NWK_HAL prototypes */

static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr);
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel);
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr);
static int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol );


/* Local prototypes */

static int8_t rf_device_register(void);
static void rf_device_unregister(void);
static void rf_lock(void);
static void rf_unlock(void);


/* Phy layer interface prototypes */

static phy_link_tx_status_e phy_tx_process_result(phyStatus_t phyResult);
static int8_t   phy_cca_tx_request(uint8_t *data, uint8_t dataLength);
static int8_t   phy_trx_state_request(phyState_t state);
static int8_t   phy_request_rx_state();
static int8_t   phy_request_idle_state();
static void     phy_force_idle_state();
static int8_t   phy_measure_channel_energy(uint8_t *energyToSet);
static int8_t   phy_pib_request(plmeSetReq_t *setRequest, plmeGetReq_t *getRequest);
static uint64_t phy_get_pib_attribute(phyPibId_t attribute);
static int8_t   phy_set_pib_attribute(phyPibId_t attribute, uint64_t value);
static int8_t   phy_set_mac_address_match(uint8_t *address);
static int8_t   phy_set_short_mac_address_match(uint8_t *address);
static int8_t   phy_set_pan_id_address_match(uint8_t *address);
static int8_t   phy_check_and_set_channel(int8_t newChannel);
static int8_t   phy_check_last_ack_frame_pending();

/*============ ARM_NWK_HAL functions ============*/

/*
 * \brief Function starts the CCA process before starting data transmission and copies the data to RF TX FIFO.
 *
 * \param data_ptr Pointer to TX data
 * \param data_length Length of the TX data
 * \param tx_handle Handle to transmission
 * \return 0 Success
 * \return -1 Busy
 */
int8_t rf_start_cca(uint8_t *data_ptr, uint16_t data_length, uint8_t tx_handle, data_protocol_e data_protocol)
{
  
    /* Check if transmitter is busy */
    if (radio_state != STATE_IDLE)
    {
        /* Return busy */
        return -1;
    }
    else
    {
        /* Check if transmitted data needs to be ACKed */
        if (*data_ptr & 0x20)
           ack_requested = 1;
        else
           ack_requested = 0;
        

        /* Start CCA and TX process */
        phy_cca_tx_request(data_ptr, data_length);
        
        /* Update current radio state */
        radio_state = STATE_TX;
    }

    /*Return success*/
    return 0;
}

/*
 * \brief Function gives the control of RF states to MAC.
 *
 * \param new_state RF state
 * \param rf_channel RF channel
 *
 * \return 0 Success
 * \return -1 Busy
 */
static int8_t rf_interface_state_control(phy_interface_state_e new_state, uint8_t rf_channel)
{

    /*
    * Sequence manager:
    * I (Idle)
    • R (Receive Sequence conditionally followed by a TxAck)
    • T (Transmit Sequence)
    • C (Standalone CCA)
    • CCCA (Continuous CCA) 
    * TR (Transmit / Receive Sequence – transmit unconditionally followed
      by either an R or RxAck)    
    */

    int8_t retVal = 0;

    switch (new_state)
    {
        /* Reset PHY driver and set to idle */
        case PHY_INTERFACE_RESET:
           
            phy_force_idle_state();
            
            radio_state = STATE_IDLE;
            
            break;

        /* Disable PHY Interface driver */
        case PHY_INTERFACE_DOWN:
            
             /* Request idle state */
            retVal = phy_request_idle_state();
           
            if (retVal == 0)
            {
                /* Set radio in low power mode */
                PhyPlmeSetPwrState(gPhyPwrDSM_c);
                radio_state = STATE_SLEEP;
            } 
           
            break;

        /* Enable PHY Interface driver */
        case PHY_INTERFACE_UP:
            
            /* Power up radio module if not already idle */
            PhyPlmeSetPwrState(gPhyPwrIdle_c); 
           
            /* Request radio RX state */
            retVal = phy_management_request(REQUEST_RX_STATE); 
            
            if (retVal == 0) 
            {
                radio_state = STATE_RX;
                phy_check_and_set_channel(rf_channel);
            }
            
            break;

        /* Enable wireless interface ED scan mode */
        case PHY_INTERFACE_RX_ENERGY_STATE:
            break;

        /* Enable Sniffer state */
        case PHY_INTERFACE_SNIFFER_STATE:
            break;
    }

    return retVal;
}


/*
 * \brief Function controls the ACK pending, channel setting and energy detection.
 *
 * \param extension_type Type of control
 * \param data_ptr Data from NET library
 *
 * \return 0 Success
 * \return -1 Busy
 */
static int8_t rf_extension(phy_extension_type_e extension_type, uint8_t *data_ptr)
{
    int8_t retVal = 0;

    switch (extension_type)
    {
        /* Control MAC pending bit for Indirect data transmission */
        case PHY_EXTENSION_CTRL_PENDING_BIT:    //TODO: Data last ack pending..

        if (*data_ptr) data_pending = true;
        else data_pending = false;
        break;

        /* Return frame pending status */
        case PHY_EXTENSION_READ_LAST_ACK_PENDING_STATUS: //TODO:

            *data_ptr = phy_get_pib_attribute(gPhyPibLastTxAckFP_c);
            break;

        /* Set channel, used for setting channel for energy scan */
        case PHY_EXTENSION_SET_CHANNEL:

            phy_check_and_set_channel(*data_ptr);
            break;

        /* Read energy on the channel */
        case PHY_EXTENSION_READ_CHANNEL_ENERGY:
           
             retVal = phy_measure_channel_energy(data_ptr);
            break;

        /* Read status of the link */
        case PHY_EXTENSION_READ_LINK_STATUS:
         
           //*data_ptr = rf_get_link_status();
            break;
    }
    return retVal;
}

/*
 * \brief Function sets the addresses to RF address filters.
 *
 * \param address_type Type of address
 * \param address_ptr Pointer to given address
 *
 * \return 0 Success
 * \return -1 Busy
 */
static int8_t rf_address_write(phy_address_type_e address_type, uint8_t *address_ptr)
{

    int8_t retVal = 0;
    phyPibId_t 
    switch (address_type)
    {
        /*Set 48-bit address*/
        case PHY_MAC_48BIT:
            
            /* Not used in this example */
            retVal = -1;
            break;

        /*Set 64-bit address*/
        case PHY_MAC_64BIT:
        
            retVal = phy_set_mac_address_match(address_ptr);
            break;

        /*Set 16-bit address*/
        case PHY_MAC_16BIT:

            retVal = phy_set_short_mac_address_match(address_ptr);
            break;

        /*Set PAN Id*/
        case PHY_MAC_PANID:

             retVal = phy_set_pan_id_address_match(address_ptr);
            break;
    }

    return retVal;
}

/*****************************************************************************/
/*****************************************************************************/


/*
 * \brief Function initialises and registers the RF driver.
 *
 * \param none
 *
 * \return rf_radio_driver_id Driver ID given by NET library
 */
static int8_t rf_device_register(void)
{
    /* Do some initialization */
    Phy_Init();
    BindToPHY(mac_instance_id);
    /* Get real MAC address */
    /* MAC is stored MSB first */
    memcpy(mac_address, (const void*)RSIM->MAC_MSB, 4);
    memcpy(&mac_address[4], (const void*)RSIM->MAC_LSB, 4);

    /* Set pointer to MAC address */
    device_driver.PHY_MAC = mac_address;
    /* Set driver Name */
    device_driver.driver_description = "KW41Z_Phy";

    /*Type of RF PHY is 2.4 GHz*/
    device_driver.link_type = PHY_LINK_15_4_2_4GHZ_TYPE;
    phy_channel_pages[0].channel_page = CHANNEL_PAGE_0;
    phy_channel_pages[0].rf_channel_configuration = &phy_2_4ghz;
    
    /*Maximum size of payload is 127*/
    device_driver.phy_MTU = 127;
    /*No header in PHY*/
    device_driver.phy_header_length = 0;
    /*No tail in PHY*/
    device_driver.phy_tail_length = 0;

    /*Set up driver functions*/
    device_driver.address_write = &rf_address_write;
    device_driver.extension = &rf_extension;
    device_driver.state_control = &rf_interface_state_control;
    device_driver.tx = &rf_start_cca;
    /*Set supported channel pages*/
    device_driver.phy_channel_pages = phy_channel_pages;
    //Nullify rx/tx callbacks
    device_driver.phy_rx_cb = NULL;
    device_driver.phy_tx_done_cb = NULL;
    device_driver.arm_net_virtual_rx_cb = NULL;
    device_driver.arm_net_virtual_tx_cb = NULL;

    /*Register device driver*/
    rf_radio_driver_id = arm_net_phy_register(&device_driver);

    return rf_radio_driver_id;
}

/*
 * \brief Function unregisters the RF driver.
 *
 * \param none
 *
 * \return none
 */
static void rf_device_unregister(void)
{
    arm_net_phy_unregister(rf_radio_driver_id);
}

static void rf_lock(void)
{
    platform_enter_critical();
}

static void rf_unlock(void)
{
    platform_exit_critical();
}


/*****************************************************************************/
/*****************************************************************************/

NanostackRfPhyKw41z::NanostackRfPhyKw41z() : NanostackRfPhy()
{
    // Do nothing
}

NanostackRfPhyKw41z::~NanostackRfPhyKw41z()
{
    rf_unregister();
}

int8_t NanostackRfPhyKw41z::rf_register()
{

    rf_if_lock();

    if (rf != NULL) {
        rf_if_unlock();
        error("Multiple registrations of NanostackRfPhyKw41z not supported");
        return -1;
    }

    int8_t radio_id = rf_device_register();
    if (radio_id < 0) {
        rf = NULL;
    } else {
        rf = this;
    }

    rf_if_unlock();
    return radio_id;
}

void NanostackRfPhyKw41z::rf_unregister()
{
    rf_if_lock();

    if (rf != this) {
        rf_if_unlock();
        return;
    }

    rf_device_unregister();
    rf = NULL;

    rf_if_unlock();
}

void NanostackRfPhyKw41z::get_mac_address(uint8_t *mac)
{
    rf_if_lock();

    memcpy(mac, mac_address, sizeof(mac_address));

    rf_if_unlock();
}

void NanostackRfPhyKw41z::set_mac_address(uint8_t *mac)
{
    rf_if_lock();

    if (NULL != rf) {
        error("NanostackRfPhyKw41z cannot change mac address when running");
        rf_if_unlock();
        return;
    }
    
    memcpy(mac_address, mac, sizeof(mac_address));

    rf_if_unlock();
}

//====================== Phy layer interface functions and callbacks =========================


/**
 * @struct PHY_Config_t
 * @brief Configuration structure for IEEE 802.15.4 PHY layer
 */
typedef struct PHY_Config {
  /**
   * Enable promiscuous mode during configuration. This can be overridden via
   * RAIL_IEEE802154_SetPromiscuousMode() afterwards.
   */
  bool promiscuousMode;
  /**
   * Set whether the device is a PAN Coordinator during configuration. This can
   * be overridden via RAIL_IEEE802154_SetPanCoordinator() afterwards.
   */
  bool isPanCoordinator;
  /**
   * Set which 802.15.4 frame types will be received, of Beacon, Data, Ack, and
   * Command. This setting can be overridden via RAIL_IEEE802154_AcceptFrames().
   */
  uint8_t framesMask;
  /**
   * Defines the default radio state after a transmit operation (transmit
   * packet, wait for ack) or a receive operation (receive packet, transmit
   * ack) finishes.
   */
  RAIL_RadioState_t defaultState;
  /**
   * Define the idleToRx and idleToTx time
   * This defines the time it takes for the radio to go into RX or TX from an
   * idle radio state
   */
  uint16_t idleTime;
  /**
   * Define the turnaround time after receiving a packet and transmitting an
   * ack and vice versa
   */
  uint16_t turnaroundTime;
  /**
   * Define the ack timeout time in microseconds
   */
  uint16_t ackTimeout;
} PHY_Config_t;


/*
 * \brief Init the PHY layer with the given parameters
 *
 * \param PHY_Config_t structure with the parameters to set
 *
 * \return 0 Success
 * \return -1 Failure
 */
 static int8_t phy_init(PHY_Config_t *param)
 {
     phy_set_pib_attribute(gPhyPibPanCoordinator_c, param->isPanCoordinator);
     phy_set_pib_attribute(gPhyPibSrcAddrEnable_c,);
     phy_set_pib_attribute(gPhyPibPromiscuousMode_c);
     phy_set_pib_attribute(gPhyPibFrameEnable_c);
     phy_set_pib_attribute(gPhyPibFrameVersion_c );
     phy_set_pib_attribute(gPhyPibRxOnWhenIdle );
     phy_set_pib_attribute(gPhyPibFrameWaitTime_c  );
     phy_set_pib_attribute(gPhyPibRxOnWhenIdle );
     phy_set_pib_attribute(gPhyPibAutoAckEnable_c );
 }

/*
 * \brief Set MAC long address to be used by the PHY's source address matching feature.
 *
 * \param Pointer to address w/ MSB first
 *
 * \return 0 Success
 * \return -1 Failure
 */
static int8_t phy_set_mac_address_match(uint8_t *address)
{
    uint64_t mac;
    for (int byte = 0; byte < 8; byte++)
    {
        mac |= address[byte] << (8 - 1 - byte);
    }

    return phy_set_pib_attribute(gPhyPibLongAddress_c, mac);
  
}

/*
 * \brief Set MAC short address to be used by the PHY's source address matching feature.
 *
 * \param Pointer to address w/ MSB first
 *
 * \return 0 Success
 * \return -1 Failure
 */
static int8_t phy_set_short_mac_address_match(uint8_t *address)
{
    uint64_t shortMac = adress[0] << 8 | address[1];

    return phy_set_pib_attribute(gPhyPibShortAddress_c, shortMac);
}

/*
 * \brief Set PAN id to be used by the PHY's source address matching feature.
 *
 * \param Pointer to address w/ MSB first
 *
 * \return 0 Success
 * \return -1 Failure
 */
static int8_t phy_set_pan_id_address_match(uint8_t *address)
{ 
    uint64_t panId = adress[0] << 8 | address[1];

    return phy_set_pib_attribute(gPhyPibPanId_c, panId);
}

/*
 * \brief 
 *
 * \param none
 *
 * \return 0 Success
 * \return -1 Failure
 */
static int8_t phy_check_and_set_channel(int8_t newChannel)
{

    int8_t retVal = 0;
    
    if (current_channel != newChannel)
    { 
        current_channel = newChannel;
            
        /* Update PHY with new channel */     
         retVal = phy_set_pib_attribute(
             gPhyPibCurrentChannel_c, newChannel);      
    }

    return retVal;
}

/*
 * \brief 
 *
 * \param none
 *
 * \return 0 Success
 * \return -1 Failure
 */
static int8_t phy_set_pib_attribute(phyPibId_t attribute, uint64_t value)
{
    plmeGetReq_t request;

    request.PibAttribute = attribute;
    request.pPibAttributeValue = value;

    return phy_pib_request(request, NULL);
}

/*
 * \brief 
 *
 * \param none
 *
 * \return 0 Success
 * \return -1 Failure
 */
static uint64_t phy_get_pib_attribute(phyPibId_t attribute)
{
    plmeGetReq_t req;
    uint64_t value;
    int8_t retVal;

    req.PibAttribute = attribute;
    req.pPibAttributeValue = &value;
    
    retVal = phy_pib_request(NULL, req);

    return retVal == 0 ? value : -1;
}

/*
 * \brief 
 *
 * \param none
 *
 * \return 0 Success
 * \return -1 Failure
 */
static int8_t phy_pib_request(plmeSetReq_t *setRequest, plmeGetReq_t *getRequest)
{
    macToPlmeMessage_t msg;
    phyStatus_t result = gPhySuccess_c;

    msg.macInstance = mac_instance_id;

    if (setRequest != NULL)
    {
        msg.msgType = gPlmeSetReq_c;
        msg.msgData.setReq = *setRequest;
    }

    else if (getRequest != NULL)
    {
        msg.msgType = gPlmeGetReq_c;
        msg.msgData.getReq = *getRequest;
    }

    else
    {
        result = gPhyInvalidParameter_c;
    }

    if (result == gPhySuccess_c)
    {
         result = MAC_PLME_SapHandler(&msg, phy_instance_id);
    }

    return result == gPhySuccess_c ? 0 : -1;
}

/*
 * \brief Register PHY callback functions
 *
 * \param none
 *
 * \return none
 */
static void phy_register_callbacks()
{
    Phy_RegisterSapHandlers(
        phy_data_msg_sap_handler, 
        phy_management_message_sap_handler, 
        phy_instance_id
    );
}

/*
 * \brief Invoke PHY layer with a CCA and following TX request
 *
 * \param none
 *
 * \return none
 */
static int8_t phy_cca_tx_request(uint8_t *data, uint8_t dataLength)
{
    macToPdDataMessage_t msg;

    msg.macInstance = mac_instance_id;
    msg.msgType = gPdDataReq_c; /* Define data request message type */

    msg.msgData.dataReq.startTime = gPhySeqStartAsap_c;
    msg.msgData.dataReq.txDuration = 0; // ?? The computed duration for the Data Request frame
    msg.msgData.dataReq.slottedTx = gPhyUnslottedTx_c; /* One CCA operation is performed */
    msg.msgData.dataReq.CCABeforeTx = gPhyCCAMode1_c; /* One CCA operation is performed */
    msg.msgData.dataReq.ackRequired = (*data & 0x20) ? gPhyRxAckRqd_c : gPhyNoAckRqd_c;
    msg.msgData.dataReq.psduLength = dataLength;
    msg.msgData.dataReq.pPsdu = data;

    /* Request to transfer message */
    MAC_PD_SapHandler(&msg, phy_instance_id);
    
    return 0;
}


/*
 * \brief Invoke PHY layer rx request.
 *
 * \param none
 *
 * \return none
 */
static int8_t phy_request_rx_state()
{
    return phy_trx_state_request(gPhy_setRxOn_c);
}

/*
 * \brief Invoke PHY layer idle request.
 *
 * \param none
 *
 * \return none
 */
static int8_t phy_request_idle_state()
{
    return phy_trx_state_request(gPhy_setRxOff_c);
}

/*
 * \brief Invoke PHY layer force idle request.
 *
 * \param none
 *
 * \return none
 */
static int8_t phy_force_idle_state()
{
    return phy_trx_state_request(gPhyForceTRxOff_c);
}

/*
 * \brief Invoke PHY layer trx state request.
 *
 * \param none
 *
 * \return none
 */
static int8_t phy_trx_state_request(phyState_t state)
{
    macToPlmeMessage_t msg;
    phyStatus_t result;

    msg.macInstance = mac_instance_id;
    msg.msgType = gPlmeSetTRxStateReq_c;

    msg.msgData.setTRxStateReq.state = state;
    msg.msgData.setTRxStateReq.slottedMode = gPhyUnslottedTx_c;
    msg.msgData.setTRxStateReq.startTime = gPhySeqStartAsap_c;
    msg.msgData.setTRxStateReq.rxDuration = 0; /* ?? If the requested state is Rx, then Rx will be enabled for rxDuration symbols.*/

    result = MAC_PLME_SapHandler(&msg, phy_instance_id);

    return result == gPhySuccess ? 0 : -1;
}

/*
 * \brief 
 *
 * \param Invoke PHY layer ED measurement request.
 *
 * \return 0  : Success
 * \return -1 : Failure
 */
static int8_t phy_measure_channel_energy(uint8_t *energyToSet)
{
    macToPlmeMessage_t msg;
    phyStatus_t result;
    uint8_t energy;
    int8_t retVal;

    msg.macInstance = mac_instance_id;
    msg.msgType = gPlmeEdReq_c;

    msg.msgData.edReq.startTime = gPhySeqStartAsap_c;

    result = MAC_PLME_SapHandler(&msg, phy_instance_id);

    if (result == gPhySuccess_c)
    {
        /* Wait for signal from SAP Handler */
        while(!lastEdCnf.newCnf);

        /* Reset signal */
        lastEdCnf.newCnf = false;

        /* Get energy level and result of energy measurement */
        *energyToSet = lastEdCnf.recvCnf.energyLevel;
        result = lastEdCnf.recvCnf.status;
    }

    return result == gPhySuccess_c ? 0 : -1;
}

/*
 * \brief Transform TX status from PHY layer to MAC layer status codes
 *
 * \param none
 *
 * \return none
 */
phy_link_tx_status_e phy_tx_process_result(phyStatus_t phyResult)
{
    phy_link_tx_status_e txStatus;

    switch(phyResult)
    {
        case gPhySuccess_c:
            
            if (ack_requested)
            {
                txStatus = PHY_LINK_TX_DONE;
            }
            else
            {
                txStatus = PHY_LINK_TX_SUCCESS;
            }
            break;
        
        case gPhyBusy_c:
            txStatus = PHY_LINK_TX_FAIL;
            break;

        case gPhyChannelBusy_c:
            txStatus = PHY_LINK_CCA_FAIL;
            break;

        case gPhyNoAck_c:
            txStatus = PHY_LINK_TX_FAIL;
            break;

        default:
            txStatus = PHY_LINK_TX_FAIL;
            break;
    }
}

/* TODO: Does PHY identify ack request or does auto ack need to be disabled?
 * \brief PHY layer data message SAP handler 
 *
 * \param none
 *
 * \return none
 */
phyStatus_t phy_data_msg_sap_handler( 
    pdDataToMacMessage_t *pMsg, 
    instanceId_t instanceId)
{ 

    /* New data has been received and is ready to be transferred to the MAC layer */
    if (pMsg->msgType == gPdDataInd_c)
    {
        /* Callback to MAC layer with received data */
        device_driver.phy_rx_cb(
            pMsg->msgData.dataInd.pPsdu,                /* Pointer to received data */
            pMsg->msgData.dataInd.psduLength,           /* Number of bytes received */
            pMsg->msgData.dataInd.ppduLinkQuality,      /* Received LQI (link quality) */
            PhyConvertLQIToRSSI(
                pMsg->msgData.dataInd.ppduLinkQuality), /* RSSI in dB */
            rf_radio_driver_id                     
        );
    }

    /* Confirmation message for a previous request to transfer data */
    else if (pMsg->msgType == gPdDataCnf_c)
    {
        /* TX confirm callback to MAC layer */
        device_driver.phy_tx_done_cb(                              
            rf_radio_driver_id,                                     
            current_tx_handle,                                         
            phy_tx_process_result(pMsg->msgData.dataCnf.status), /* TX confirm message */
            1,                                                      /* CCA retries */
            1                                                       /* TX retries */
        );
    }

    return gPhySuccess_c;
}

/*
 * \brief PHY layer management message SAP handler
 *
 * \param none
 *
 * \return none
 */
phyStatus_t phy_management_message_sap_handler(
    plmeToMacMessage_t *pMsg, 
    instanceId_t instanceId)
{

    switch(pMsg->msgType)
    {
        case gPlmeCcaCnf_c:
        // Not implemented
        break;

        /* Channel energy measurement confirmation message */
        case gPlmeEdCnf_c:
        
        lastEdCnf.recvCnf = *confMsg;   /* Update local last received energy measurement */
        lastEdCnf.newCnf = true;        /* Signal that new measurement is available */
        break;
    }
    
    return gPhySuccess_c;
}