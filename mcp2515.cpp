#include "mcp2515.h"

const struct MCP2515::TXBn_REGS MCP2515::TXB[MCP2515::N_TXBUFFERS] = {
    {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
};

const struct MCP2515::RXBn_REGS MCP2515::RXB[N_RXBUFFERS] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF}
};

MCP2515::MCP2515(const uint8_t _CS, const uint8_t _SPI_BUS)
{
    if(_SPI_BUS == 1) {
        SPI_BUS = SPI1;
    } else {
        SPI_BUS = SPI;
    }

    SPICS = _CS;
    pinMode(SPICS, OUTPUT);
    endSPI();
}

void MCP2515::startSPI() {
    SPI_BUS.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(SPICS, LOW);
}

void MCP2515::endSPI() {
    digitalWrite(SPICS, HIGH);
    SPI_BUS.endTransaction();
}

MCP2515::ERROR MCP2515::reset(void)
{
    startSPI();
    SPI_BUS.transfer(INSTRUCTION_RESET);
    endSPI();

    delay(10);

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    setRegisters(MCP_TXB0CTRL, zeros, 14);
    setRegisters(MCP_TXB1CTRL, zeros, 14);
    setRegisters(MCP_TXB2CTRL, zeros, 14);

    // initializes the registers to base settings 
    setRegister(MCP_RXB0CTRL, 0);   
    setRegister(MCP_RXB1CTRL, 0);  

    // Initialize the CANINTE register, 0x2B is the interrupt enable register. 
    // Note that the CANINTF register, 0x2C is similar, but is the flag register       
    // CANINTF_RX0IF = 0X01 = Interrupt when message was received in RXB0
        // Note that CANINTF and CANINTE_XXXXX are the same, so we can use the XXXXF and XXXXE interchangably 
    // CANINTF_RX1IF = 0X02 = Interrupt when message was received in RXB1
    // CANINTF_ERRIF = Interrupt if there is an error flag 
    // CANINtF_MERRF = Interrupt if there is an message error flag 
    setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // not sure if this is actually doing what we want... what is the series of OR-ed inputs doing??  

    
    //modifyRegister(MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT, RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT);
    //modifyRegister(MCP_RXB1CTRL, RXBnCTRL_RXM_MASK, RXBnCTRL_RXM_STDEXT);
    /*
    // Just for sanity, we're going to print the binary representations of the register configuratiosn 
    Serial.print("RXB0 Configuration; Mask = ");
    Serial.print(RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT, DEC);
    Serial.print(" Data = ");
    Serial.println( RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT, DEC);
    Serial.print("RXB1 Configuration; Mask = ");
    Serial.print(RXBnCTRL_RXM_MASK, DEC);
    Serial.print(" Data = ");

    */


    // Modify the registers to let ANYTHING THROUGH

    // Alternative setting, writing to 0x60 for RXB0 as 0x64
    modifyRegister(MCP_RXB0CTRL, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT, RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT);
    // Alternative setting, writing to 0x70 for RXB1 as 0x60
    modifyRegister(MCP_RXB1CTRL, RXBnCTRL_RXM_MASK,RXBnCTRL_RXM_MASK);

    /*
    Serial.print("RXB0 Configuration; Mask = ");
    Serial.print(RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT, DEC);
    Serial.print(" Data = ");
    Serial.println(RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT, DEC);
    Serial.print("RXB1 Configuration; Mask = ");
    Serial.print(RXBnCTRL_RXM_MASK, DEC);
    Serial.print(" Data = ");
    Serial.println(RXBnCTRL_RXM_MASK, DEC);
    */ 
    
    

    // clear filters and masks
    /*RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i=0; i<6; i++) {
        ERROR result = setFilter(filters[i], true, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i=0; i<2; i++) {
        ERROR result = setFilterMask(masks[i], true, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }*/

    return ERROR_OK;
}

uint8_t MCP2515::readRegister(const REGISTER reg)
{
    startSPI();
    SPI_BUS.transfer(INSTRUCTION_READ);
    SPI_BUS.transfer(reg);
    uint8_t ret = SPI_BUS.transfer(0x00);
    endSPI();

    return ret;
}

void MCP2515::readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n)
{
    startSPI();
    SPI_BUS.transfer(INSTRUCTION_READ);
    SPI_BUS.transfer(reg);
    // mcp2515 has auto-increment of address-pointer
    for (uint8_t i=0; i<n; i++) {
        values[i] = SPI_BUS.transfer(0x00);
    }
    endSPI();
}

void MCP2515::setRegister(const REGISTER reg, const uint8_t value)
{
    startSPI();
    SPI_BUS.transfer(INSTRUCTION_WRITE);
    SPI_BUS.transfer(reg);
    SPI_BUS.transfer(value);
    endSPI();
}

void MCP2515::setRegisters(const REGISTER reg, const uint8_t values[], const uint8_t n)
{
    startSPI();
    SPI_BUS.transfer(INSTRUCTION_WRITE);
    SPI_BUS.transfer(reg);
    for (uint8_t i=0; i<n; i++) {
        SPI_BUS.transfer(values[i]);
    }
    endSPI();
}

void MCP2515::modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data)
{
    startSPI();
    SPI_BUS.transfer(INSTRUCTION_BITMOD);  
    SPI_BUS.transfer(reg);
    SPI_BUS.transfer(mask);
    SPI_BUS.transfer(data);  
    endSPI();
}

uint8_t MCP2515::getStatus(void)
{
    startSPI();
    SPI_BUS.transfer(INSTRUCTION_READ_STATUS);
    uint8_t i = SPI_BUS.transfer(0x00);
    endSPI();

    return i;
}

MCP2515::ERROR MCP2515::setConfigMode()
{
    return setMode(CANCTRL_REQOP_CONFIG);
}

MCP2515::ERROR MCP2515::setListenOnlyMode()
{
    return setMode(CANCTRL_REQOP_LISTENONLY);
}

MCP2515::ERROR MCP2515::setSleepMode()
{
    return setMode(CANCTRL_REQOP_SLEEP);
}

MCP2515::ERROR MCP2515::setLoopbackMode()
{
    return setMode(CANCTRL_REQOP_LOOPBACK);
}

MCP2515::ERROR MCP2515::setNormalMode()
{
    return setMode(CANCTRL_REQOP_NORMAL);
}

MCP2515::ERROR MCP2515::setMode(const CANCTRL_REQOP_MODE mode)
{
    modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

    unsigned long endTime = millis() + 10;
    bool modeMatch = false;
    while (millis() < endTime) {
        uint8_t newmode = readRegister(MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        modeMatch = newmode == mode;

        if (modeMatch) {
            break;
        }
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;

}

MCP2515::ERROR MCP2515::setBitrate(const CAN_SPEED canSpeed)
{
    return setBitrate(canSpeed, MCP_16MHZ);
}

MCP2515::ERROR MCP2515::setBitrate(const CAN_SPEED canSpeed, CAN_CLOCK canClock)
{
    ERROR error = setConfigMode();
    if (error != ERROR_OK) {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
        case (MCP_8MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5KBPS
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10KBPS
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20KBPS
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

            case (CAN_31K25BPS):                                            //  31.25KBPS
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

            case (CAN_33KBPS):                                             //  33.33KBPS
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_16MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

            case (CAN_33KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

            case (CAN_83K3BPS):                                              //  83.3333Kbps
            cfg1 = MCP_16MHz_83k3BPS_CFG1;
            cfg2 = MCP_16MHz_83k3BPS_CFG2;
            cfg3 = MCP_16MHz_83k3BPS_CFG3;
            break; 

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_20MHZ):
        switch (canSpeed)
        {
            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        default:
        set = 0;
        break;
    }


    if (set) {
        setRegister(MCP_CNF1, cfg1);
        setRegister(MCP_CNF2, cfg2);
        setRegister(MCP_CNF3, cfg3);
        return ERROR_OK;
    }
    else {
        return ERROR_FAIL;
    }
}

void MCP2515::prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    // start with the bottom 2 bytes, which is the case for an 11 bit mafk 
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext) {
        
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);        // copies the lower byte 
        //MSG: XXXX

        buffer[MCP_EID8] = (uint8_t) (canid >> 8);          // copies the upper byte by shifting 
        //MSG: XXXX    XXXX
        
        canid = (uint16_t)(id >> 16);                       // now pop the rest of the id in there 

        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);        // the lower section of the ID, at least the 2 bytes 
        // MSG: --XX XXXX    XXXX        
        
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);    // this is the remaining bits with 0001 1100 then shifted by 3 to the left. 

        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;                 // yep, checks out 

        // but what about the SRR? 
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5);

    // if not extended, shift right by three? Why is this necessary? 
    } else {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

MCP2515::ERROR MCP2515::setFilterMask(const MASK mask, const bool ext, const uint32_t ulData)
{
    ERROR res = setConfigMode();
    if (res != ERROR_OK) {
        return res;
    }
    
    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return ERROR_FAIL;
    }

    setRegisters(reg, tbufdata, 4);
    
    return ERROR_OK;
}

MCP2515::ERROR MCP2515::setFilter(const RXF num, const bool ext, const uint32_t ulData)
{
    ERROR res = setConfigMode();
    if (res != ERROR_OK) {
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);
    setRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}


// When sending a message, we are writing to 13 registers in the MCP2515; 
//  TXBnSIDH (occupying 0x31, 0x41 and 0x51)    --> these are the 8lsbs of the 32 bit integer 
//  TXBnSIDL (occupying 0x32, 0x42 and 0x52)    --> these are the 16-9lsbs of the 32 bit integer
//  TXBnEID8 (occupying 0x33, 0x43 and 0x53)    --> these are the 24-17lsbs of the 32 bit integer 
//  TXBnEID0 (occupying 0x34, 0x44 and 0x54)    --> these are the 32-25lsbs of the 32 bit integer 
//  TXBnDLC  (occupying 0x35, 0x45 and 0x55)    --> the 4lsbs allow 0-8 bytes <6> is the rtr flag 
//  TXBnDM   these are the next sequence of byte registers, that are populated as data byyes 

// All of this information is recorded in the header file, but we need to
// ensure that these are in fact getting written properly (I think they are)
// without some kind of weird masking procedure before their write muddling 
// their values. 

// It may be necessary to modify the send message to include a check for extended
// frame and use this check to set the registers accordingly. 
// We should see what the datasheet reccommends, if anything. 

MCP2515::ERROR MCP2515::sendMessage(const TXBn txbn, const struct can_frame *frame)
{
    const struct TXBn_REGS *txbuf = &TXB[txbn];         // which buffer are we writing too 
  
    uint8_t data[13];                                   // structure to catch 13 bytes of data 

    // the maximul value a standard ID can take is 4095, this should be a requisite even without the flag  
    bool ext = (frame->can_id > 4095 || frame-> can_id & CAN_EFF_FLAG);          
    
    bool rtr = (frame->can_id & CAN_RTR_FLAG);

    // This mask wipes any of the higher-order bits that aren't relevant 
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    // splits the id up into the apropriate buffer slots 
    prepareId(data, ext, id);

    // set the rtr bit if need be 
    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    // copies the data bits to the data portion of the array, simple. 
    memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    // set the registers in the MCP
    setRegisters(txbuf->SIDH, data, 5 + frame->can_dlc);

    // 
    modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

    return ERROR_OK;
}

MCP2515::ERROR MCP2515::sendMessage(const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    TXBn txBuffers[N_TXBUFFERS] = {TXB0, TXB1, TXB2};

    for (int i=0; i<N_TXBUFFERS; i++) {
        const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = readRegister(txbuf->CTRL);
        if ( (ctrlval & TXB_TXREQ) == 0 ) {
            return sendMessage(txBuffers[i], frame);
        }
    }

    return ERROR_FAILTX;
}

MCP2515::ERROR MCP2515::readMessage(const RXBn rxbn, struct can_frame *frame)
{
    const struct RXBn_REGS *rxb = &RXB[rxbn];       // which register are we reading from?

    uint8_t tbufdata[5]; // this is a 5-byte buffer     

    readRegisters(rxb->SIDH, tbufdata, 5); // we want to read the first 5 bits from dis reg. 

    uint32_t id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5); 
    //  id  ---- ---- ---- ---- ---- ---- ---- ----
    //  sidh---- ---- ---- ---- ---- -aaa aaaa a--- ("a")
    //  sidl---- ---- ---- ---- ---- -aaa aaaa abbb ("b")


    if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
        id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        //  id---- ---- ---- ---- ---- -aaa aaaa abbb
        //id<2---- ---- ---- ---- --aa aaaa aabb bbbb &3 checks out, keeps the two lower bits. 
        id = (id<<8) + tbufdata[MCP_EID8]; // "c"
        //  id---- ---- ---a aaaa aaab bb11 cccc cccc
        id = (id<<8) + tbufdata[MCP_EID0]; // "d"
        //  id---a aaaa aaab bb11 cccc cccc dddd dddd 
        // id |= CAN_EFF_FLAG;
        // now we're adding the CAN extended flag??? 
    }

    // this is taking in the DLC..good 
    uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return ERROR_FAIL;
    }

    // handle the request for response 
    uint8_t ctrl = readRegister(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    // this is reading MULTIPLE registers basen on the DLC.
    readRegisters(rxb->DATA, frame->data, dlc);

    // I believe that this clears the receive code.
    // modifyRegister(Target, Source, Destination); 
    // modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0); // Original
    modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0); // Original

    // We're going to clear the error flags as well, if they exist,this might be causing problems.  
    modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0); // Original
    modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0); // Original

 


    return ERROR_OK;
}

MCP2515::ERROR MCP2515::readMessage(struct can_frame *frame)
{
    ERROR rc;
    uint8_t stat = getStatus();

    if ( stat & STAT_RX0IF ) {
        rc = readMessage(RXB0, frame);
    } else if ( stat & STAT_RX1IF ) {
        rc = readMessage(RXB1, frame);
    } else {
        rc = ERROR_NOMSG;
    }

    return rc;
}

MCP2515::ERROR MCP2515::readrxb0(struct can_frame *frame) {
    if(getStatus() & STAT_RX0IF) {
        // Instead of just calling read message, lets use the Read Rx Buffer Instruction
        uint8_t READ_RXB0 = 144; 
        uint8_t in_buff[8];
        uint8_t empty_buff[13];
        startSPI();
        SPI_BUS.transfer(READ_RXB0);
        SPI_BUS.transfer(empty_buff, in_buff, 13); // doesn't matter what buf is, doesn't matter 
        // lets see what this gives us.. 

        return readMessage(RXB0, frame);
    } else {

        uint8_t READ_RXB1 = 148; 

        Serial.println("No message received.. ");
        return ERROR_NOMSG;
    }
}

MCP2515::ERROR MCP2515::readrxb1(struct can_frame *frame) {
    if(getStatus() & STAT_RX1IF) {
        return readMessage(RXB1, frame);
    } else {
        Serial.println("No message received.." );
        return ERROR_NOMSG;
    }
}


// returns true -- perhaps this would be more effective if 
// it returned an int, 0 for nothing, 1 for RXB0 and 2 for RXB1? 
// original method, for reference.

bool MCP2515::checkReceive(void)
{
    uint8_t res = getStatus();
    if ( res & STAT_RXIF_MASK ) {
        return true;
    } else {
        return false;
    }
}


bool MCP2515::checkError(void)
{
    uint8_t eflg = getErrorFlags();

    if ( eflg & EFLG_ERRORMASK ) {
        return true;
    } else {
        return false;
    }
}

uint8_t MCP2515::getErrorFlags(void)
{
    return readRegister(MCP_EFLG);
}

void MCP2515::clearRXnOVRFlags(void)
{
	modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t MCP2515::getInterrupts(void)
{
    return readRegister(MCP_CANINTF);
}

void MCP2515::clearInterrupts(void)
{
    setRegister(MCP_CANINTF, 0);
}

uint8_t MCP2515::getInterruptMask(void)
{
    return readRegister(MCP_CANINTE);
}

void MCP2515::clearTXInterrupts(void)
{
    modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void MCP2515::clearRXnOVR(void)
{
	uint8_t eflg = getErrorFlags();
	if (eflg != 0) {
		clearRXnOVRFlags();
		clearInterrupts();
		//modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
	}
	
}

void MCP2515::clearMERR()
{
	//modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
	//clearInterrupts();
	modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
}
