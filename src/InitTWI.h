#ifndef _InitTWI_h
#define _InitTWI_h
#include "sam3x8e.h"

static const uint32_t RECV_TIMEOUT = 100000;
static const uint32_t XMIT_TIMEOUT = 100000;

void TWIbegin();


void TWI_ConfigureMaster( Twi* pTwi, uint32_t dwTwCk, uint32_t dwMCk );

void TWI_SetClock( Twi *pTwi, uint32_t dwTwCk, uint32_t dwMCk );

void TWI_ConfigureSlave(Twi *pTwi, uint8_t slaveAddress);


void TWI_Disable(Twi *pTwi);

void TWI_Stop( Twi *pTwi );

void TWI_StartRead(
    Twi *pTwi,
    uint8_t address,
    uint32_t iaddress,
    uint8_t isize);

uint8_t TWI_ReadByte(Twi *pTwi);

void TWI_WriteByte(Twi *pTwi, uint8_t byte);

void TWI_StartWrite(
    Twi *pTwi,
    uint8_t address,
    uint32_t iaddress,
    uint8_t isize,
    uint8_t byte);

uint8_t TWI_ByteReceived(Twi *pTwi);

uint8_t TWI_ByteSent(Twi *pTwi);

uint8_t TWI_TransferComplete(Twi *pTwi);

void TWI_EnableIt(Twi *pTwi, uint32_t sources);

void TWI_DisableIt(Twi *pTwi, uint32_t sources);

uint32_t TWI_GetStatus(Twi *pTwi);

uint32_t TWI_GetMaskedStatus(Twi *pTwi);

void TWI_SendSTOPCondition(Twi *pTwi);

void TWIbegin(){
	//Disable the Write Protect Mode
	PMC->PMC_WPMR &= ~(PMC_WPMR_WPEN);
	PIOB->PIO_WPMR &= ~(PIO_WPMR_WPEN);
	//Disable PIO Controller
	PIOB->PIO_PDR |= PIO_PDR_P12|PIO_PDR_P13;
	//Peripheral A selected by default
	PIOB->PIO_ABSR &= ~(PIO_ABSR_P12|PIO_ABSR_P13);
	//Enable TWI peripheral Clock
	PMC->PMC_PCER0 |= (1u<<ID_TWI1);
	
	PIOA->PIO_WPMR &= ~(PIO_WPMR_WPEN);
	//Disable PIO Controller
	PIOA->PIO_PDR |= PIO_PDR_P17|PIO_PDR_P18;
	//Peripheral A selected by default
	PIOA->PIO_ABSR &= ~(PIO_ABSR_P17|PIO_ABSR_P18);
	//Enable TWI peripheral Clock
	PMC->PMC_PCER0 |= (1u<<ID_TWI0);
}

void TWI_ConfigureMaster( Twi* pTwi, uint32_t dwTwCk, uint32_t dwMCk )
{
	// Disable PDC channel
	pTwi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
	
    /* SVEN: TWI Slave Mode Enabled */
    pTwi->TWI_CR = TWI_CR_SVEN ;

    TWI_Disable(pTwi);

    /* Set master mode */
    pTwi->TWI_CR = TWI_CR_MSEN ;

    /* Configure clock */
    TWI_SetClock(pTwi, dwTwCk, dwMCk);
}


void TWI_SetClock( Twi *pTwi, uint32_t dwTwCk, uint32_t dwMCk )
{
    uint32_t dwCkDiv = 0 ;
    uint32_t dwClDiv ;
    uint32_t dwOk = 0 ;

    /* Configure clock */
    while ( !dwOk )
    {
        dwClDiv = ((dwMCk / (2 * dwTwCk)) - 4) / (1<<dwCkDiv) ;

        if ( dwClDiv <= 255 )
        {
            dwOk = 1 ;
        }
        else
        {
            dwCkDiv++ ;
        }
    }

    pTwi->TWI_CWGR = 0 ;
    pTwi->TWI_CWGR = (dwCkDiv << 16) | (dwClDiv << 8) | dwClDiv ;
}

void TWI_ConfigureSlave(Twi *pTwi, uint8_t slaveAddress)
{
    uint32_t i;

    TWI_Disable(pTwi);

    /* Configure slave address. */
    pTwi->TWI_SMR = 0;
    pTwi->TWI_SMR = TWI_SMR_SADR(slaveAddress);

    /* SVEN: TWI Slave Mode Enabled */
    pTwi->TWI_CR = TWI_CR_SVEN;

    /* Wait at least 10 ms */
    for (i=0; i < 1000000; i++);
}

void TWI_Disable(Twi *pTwi)
{
    uint32_t i;

    /* TWI software reset */
    pTwi->TWI_CR = TWI_CR_SWRST;
    pTwi->TWI_RHR;

    /* Wait at least 10 ms */
    for (i=0; i < 1000000; i++);

    /* TWI Slave Mode Disabled, TWI Master Mode Disabled*/
    pTwi->TWI_CR = TWI_CR_SVDIS | TWI_CR_MSDIS;
}

void TWI_Stop( Twi *pTwi )
{
    pTwi->TWI_CR = TWI_CR_STOP;
}

void TWI_StartRead(
    Twi *pTwi,
    uint8_t address,
    uint32_t iaddress,
    uint8_t isize)
{

    /* Set slave address and number of internal address bytes. */
    pTwi->TWI_MMR = 0;
    pTwi->TWI_MMR = (isize << 8) | TWI_MMR_MREAD | (address << 16);

    /* Set internal address bytes */
    pTwi->TWI_IADR = 0;
    pTwi->TWI_IADR = iaddress;

    /* Send START condition */
    pTwi->TWI_CR = TWI_CR_START;
}

uint8_t TWI_ReadByte(Twi *pTwi)
{
	return pTwi->TWI_RHR;
}

void TWI_WriteByte(Twi *pTwi, uint8_t byte)
{
    pTwi->TWI_THR = byte;
}

void TWI_StartWrite(
    Twi *pTwi,
    uint8_t address,
    uint32_t iaddress,
    uint8_t isize,
    uint8_t byte)
{
    /* Set slave address and number of internal address bytes. */
    pTwi->TWI_MMR = 0;
    pTwi->TWI_MMR = (isize << 8) | (address << 16);

    /* Set internal address bytes. */
    pTwi->TWI_IADR = 0;
    pTwi->TWI_IADR = iaddress;

    /* Write first byte to send.*/
    TWI_WriteByte(pTwi, byte);
}

uint8_t TWI_ByteReceived(Twi *pTwi)
{
    return ((pTwi->TWI_SR & TWI_SR_RXRDY) == TWI_SR_RXRDY);
}

uint8_t TWI_ByteSent(Twi *pTwi)
{
    return ((pTwi->TWI_SR & TWI_SR_TXRDY) == TWI_SR_TXRDY);
}

uint8_t TWI_TransferComplete(Twi *pTwi)
{
    return ((pTwi->TWI_SR & TWI_SR_TXCOMP) == TWI_SR_TXCOMP);
}

void TWI_EnableIt(Twi *pTwi, uint32_t sources)
{
    pTwi->TWI_IER = sources;
}

void TWI_DisableIt(Twi *pTwi, uint32_t sources)
{
    pTwi->TWI_IDR = sources;
}

uint32_t TWI_GetStatus(Twi *pTwi)
{
    return pTwi->TWI_SR;
}

uint32_t TWI_GetMaskedStatus(Twi *pTwi)
{
    uint32_t status;
	
    status = pTwi->TWI_SR;
    status &= pTwi->TWI_IMR;

    return status;
}

void TWI_SendSTOPCondition(Twi *pTwi)
{
    pTwi->TWI_CR |= TWI_CR_STOP;
}

static inline bool TWI_FailedAcknowledge(Twi *pTwi) {
	return pTwi->TWI_SR & TWI_SR_NACK;
}

static inline bool TWI_WaitTransferComplete(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
		return false;

		if (--_timeout == 0)
		return false;
	}
	return true;
}

static inline bool TWI_WaitByteSent(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXRDY) != TWI_SR_TXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
		return false;

		if (--_timeout == 0)
		return false;
	}

	return true;
}

static inline bool TWI_WaitByteReceived(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_RXRDY) != TWI_SR_RXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
		return false;

		if (--_timeout == 0)
		return false;
	}

	return true;
}

static inline bool TWI_STATUS_SVREAD(uint32_t status) {
	return (status & TWI_SR_SVREAD) == TWI_SR_SVREAD;
}

static inline bool TWI_STATUS_SVACC(uint32_t status) {
	return (status & TWI_SR_SVACC) == TWI_SR_SVACC;
}

static inline bool TWI_STATUS_GACC(uint32_t status) {
	return (status & TWI_SR_GACC) == TWI_SR_GACC;
}

static inline bool TWI_STATUS_EOSACC(uint32_t status) {
	return (status & TWI_SR_EOSACC) == TWI_SR_EOSACC;
}

static inline bool TWI_STATUS_NACK(uint32_t status) {
	return (status & TWI_SR_NACK) == TWI_SR_NACK;
}



#endif