/***************************************************************************
 *   Copyright (C) 2022 - 2023 by Federico Amedeo Izzo IU2NUO,             *
 *                                Niccolò Izzo IU2KIN                      *
 *                                Frederik Saraci IU2NRO                   *
 *                                Silvano Seva IU2KWO                      *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 *                                                                         *
 *   Modified by KD0OSS for P25 on Module17/OpenRTX                        *
 ***************************************************************************/
#include <interfaces/platform.h>
#include <interfaces/delays.h>
#include <interfaces/audio.h>
#include <interfaces/radio.h>
#include <OpMode_P25.hpp>
#include <cstdint>
#include <imbe_audio_codec.h>
#include <errno.h>
#include <rtx.h>
#include <state.h>
#include <settings.h>
#include <DSTAR/RingBuffer.h>
//#include <drivers/usb_vcom.h>

#ifdef PLATFORM_MOD17
#include <drivers/USART3_MOD17.h> // for debugging
#include <drivers/usb_vcom.h>
#include <calibInfo_Mod17.h>

extern mod17Calib_t mod17CalData;
#endif

CRingBuffer<uint8_t>  p25LDUBuffer(2180);
CRingBuffer<uint8_t>  p25PacketBuffer(2000);

extern CRingBuffer<uint16_t> p25_txBuffer;

extern bool host_found;
extern bool p25_tx;
extern bool txing;

extern bool host_found;

OpMode_P25::OpMode_P25():
startRx(false),
startTx(false),
dataValid(false),
locked(false),
invertTxPhase(false),
invertRxPhase(false)
{
	m_srcid = 0;
	m_dstid = 0;
}

OpMode_P25::~OpMode_P25()
{
    disable();
}

void OpMode_P25::reset()
{
	p25_io.reset();
}

void OpMode_P25::enable()
{
    dataValid    = false;
    startRx      = true;
    startTx      = false;
	p25_tx       = false;

    imbe_init();
    p25_io.start();
    p25_io.init_tx();
}

void OpMode_P25::disable()
{
	locked       = false;
    dataValid    = false;
    startRx      = false;
    startTx      = false;

    p25_io.terminate_tx();
    p25_io.terminate();
	p25_io.stopBasebandSampling();
    imbe_stop(rxAudioPath);
    platform_ledOff(GREEN);
    platform_ledOff(RED);
    audioPath_release(rxAudioPath);
    audioPath_release(txAudioPath);
    imbe_terminate();
    radio_disableRtx();
}

void OpMode_P25::update(rtxStatus_t *const status, const bool newCfg)
{
    (void) newCfg;

    #if defined(PLATFORM_MD3x0) || defined(PLATFORM_MDUV3x0)
    //
    // Invert TX phase for all MDx models.
    // Invert RX phase for MD-3x0 VHF and MD-UV3x0 radios.
    //
    const hwInfo_t* hwinfo = platform_getHwInfo();
    invertTxPhase = true;
    if(hwinfo->vhf_band == 1)
        invertRxPhase = true;
    else
        invertRxPhase = false;
    invertTxPhase = state.settings.p25_tx_invert;
    invertRxPhase = state.settings.p25_rx_invert;
    #elif defined(PLATFORM_MOD17)
    if(!host_found) // Check if P25 host has connected
    {
      	uint8_t buf[2];
       	uint8_t count = vcom_readBlock((uint8_t*)buf, 2);
       	if (count == 2)
       	{
       		if (buf[0] == 0x61 && buf[1] == 0x03)
       			host_found = true;
       	}
  //     	sleepFor(0, 100);
    }

//
// Get phase inversion settings from calibration.
//
    invertTxPhase = !(mod17CalData.bb_tx_invert == 1) ? true : false;
    invertRxPhase = !(mod17CalData.bb_rx_invert == 1) ? true : false;
    #elif defined(PLATFORM_CS7000)
    invertTxPhase = false;
    invertRxPhase = true;
    #elif  defined(PLATFORM_CS7000P)
    invertTxPhase = state.settings.p25_tx_invert;
    invertRxPhase = state.settings.p25_rx_invert;
    #elif defined(PLATFORM_DM1701)
    invertTxPhase = false;
    invertRxPhase = false;
    #endif

    // Main FSM logic
    switch(status->opStatus)
    {
        case OFF:
            offState(status);
            break;

        case RX:
            rxState(status);
            break;
#ifdef PLATFORM_CS7000P
        case TX:
            txState(status);
            break;
#endif
        default:
            break;
    }

    // Led control logic
    switch(status->opStatus)
    {
        case RX:

            if(dataValid)
                platform_ledOn(GREEN);
            else
                platform_ledOff(GREEN);
            break;

        case TX:
            platform_ledOff(GREEN);
            platform_ledOn(RED);
            break;

        default:
            platform_ledOff(GREEN);
            platform_ledOff(RED);
            break;
    }
}

void OpMode_P25::offState(rtxStatus_t *const status)
{
    radio_disableRtx();

    if(startRx)
    {
        status->opStatus = RX;
        return;
    }

    if(platform_getPttStatus() && (status->txDisable == 0))
    {
        startTx = true;
        status->opStatus = TX;
        return;
    }

    // Sleep for 30ms if there is nothing else to do in order to prevent the
    // rtx thread looping endlessly and locking up all the other tasks
    sleepFor(0, 30);
}

void OpMode_P25::rxState(rtxStatus_t *const status)
{
    if(startRx)
    {
    	p25PacketBuffer.reset();
        p25_io.startBasebandSampling();
        pthSts = PATH_CLOSED;

        radio_enableRx();

        startRx = false;
        locked = false;
    }

    locked = p25_io.update(invertRxPhase);
	dataValid = p25_io.isValid();

    if(locked && dataValid)
    {
    	if(imbe_running() == false)
    	{
    		pthSts = audioPath_getStatus(rxAudioPath);
    		if(pthSts == PATH_CLOSED)
    		{
    			rxAudioPath = audioPath_request(SOURCE_MCU, SINK_SPK, PRIO_RX);
    			pthSts = audioPath_getStatus(rxAudioPath);
    		}
    		imbe_startDecode(rxAudioPath);
    	}

  	    if(p25PacketBuffer.getSpace() > 99)
    		p25_io.decode();

    	status->P25_SrcId = p25_io.getSrcId();
    	status->P25_DstId = p25_io.getDstId();
    	status->lsfOk = true;
/*
        uint8_t buf[17];
        buf[0] = 0x61;
        buf[1] = 0x00;
        buf[2] = 0x0d;
        buf[3] = 0x03; // Type = imbe packets for imbeDecoder
        buf[4] = 0x01;
        buf[5] = 0x48;

        while(p25PacketBuffer.getData() >= 11)
        {
        	for (int i=0;i<11;i++)
        	{
        		p25PacketBuffer.get(buf[6+i]);
        	}
        	if(host_found)
        	{
        		vcom_writeBlock((uint8_t*)buf, 17);
        	}
        } */
    }
    else
    	p25LDUBuffer.reset();

#if !defined(PLATFORM_CS7000P) && !defined(PLATFORM_MD3x0)
    if(platform_getPttStatus() && host_found)
#else
    if(platform_getPttStatus())
#endif
    {
    	locked = false;
    	p25_io.stopBasebandSampling();
    	status->opStatus = OFF;
    }

    // Force invalidation of LSF data as soon as lock is lost (for whatever cause)
    if(locked == false)
    {
   // 	while (p25LDUBuffer.getData() >= P25_LDU_FRAME_LENGTH_BYTES + 1U)
    //		p25_io.decode();
    	p25LDUBuffer.reset();
    	if(pthSts != PATH_CLOSED)
    	{
    	    imbe_stop(rxAudioPath);
    	    audioPath_release(rxAudioPath);
    	}
    	dataValid = false;
    	status->lsfOk = false;
    }
}

void OpMode_P25::txState(rtxStatus_t *const status)
{
#if !defined(PLATFORM_CS7000P) && !defined(PLATFORM_MD3x0)
	if(!host_found) // receive only
	{
		sleepFor(0, 30);
		return;
	}
#endif
	if(startTx)
	{
		end_tx = false;
		delay = 0;
    	p25PacketBuffer.reset();
        txAudioPath = audioPath_request(SOURCE_MIC, SINK_MCU, PRIO_TX);
		imbe_startEncode(txAudioPath);
		sleepFor(0, 100); // wait for some tx audio bytes
	    txing = true;
		p25_tx = false;
		startTx = false;
		radio_enableTx();
		p25_io.start_tx(invertTxPhase);
		p25_io.setSrcId(state.settings.p25_srcId);
		p25_io.setDstId(state.settings.p25_dstId);
		p25_io.writeSyncFrames();
	//	p25_io.writeSyncFrames();
		p25_io.createTxHeader();
	}
#ifdef PLATFORM_MOD17
	if(vcom_bytesReady() >= 17)
	{
		// look for start of IMBE packet
		uint8_t buf[16];
		vcom_readBlock(buf, 1);
		if(buf[0] == 0x61)
		{
			vcom_readBlock(buf, 16);
            for(size_t i = 5; i < 16; i++)
				p25PacketBuffer.put(buf[i]);
#endif
            if(p25PacketBuffer.getData() >= 198) // wait for 360ms of encoded audio
			{
				p25_io.createTxLDU1();
				p25_io.createTxLDU2();
			}
            else
            	sleepFor(0, 2);
#ifdef PLATFORM_MOD17
		}
	}
	else
		sleepFor(0, 1);
#endif

	if(platform_getPttStatus() == false && txing && !end_tx)
	{
		end_tx = true;
		delay = 0;
		imbe_stop(txAudioPath);
		audioPath_release(txAudioPath);
	}

	if(end_tx == true)
		delay++;

	if(delay >= 4)
	{
		delay = 0;
		end_tx = false;
		p25_io.createTxLDU1();
		p25_io.createTxLDU2();
		p25_io.createTxTerminator();
		sleepFor(0, 20);
    	txing = false;
		p25_io.stop_tx();
		p25_tx = false;
		startRx = true;
		status->opStatus = OFF;
	}
}
