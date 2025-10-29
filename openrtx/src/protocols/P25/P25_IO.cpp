/*
 *   Copyright (C) 2009-2016 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *   (2025) Modified by KD0OSS for use in OpenRTX
 */
//#ifdef CONFIG_P25

#include <P25/P25_IO.hpp>
#include <DSTAR/RingBuffer.h>
#include <interfaces/platform.h>
#include <interfaces/delays.h>
#include <audio_stream.h>
#include <threads.h>
#include <string.h>
#include <cstdint>
#include <string>
#include <dsp.h>
#include <crc.h>
#include <state.h>
#include <P25/P25Defines.h>
#include <P25/P25Utils.h>
#include <P25/BCH.h>
#include <P25/Golay24128.h>
#include <P25/Hamming.h>
#include <P25/AMBEFEC.h>
#include <P25/P25_IO.hpp>
//#include <drivers/usb_vcom.h>

#ifdef PLATFORM_MOD17
#include <calibInfo_Mod17.h>
#include <drivers/USART3_MOD17.h> // used for debugging
extern mod17Calib_t         mod17CalData;
#endif

using namespace std;

// One symbol boxcar filter
static q15_t   BOXCAR5_FILTER[] = {12000, 12000, 12000, 12000, 12000, 0};
const uint16_t BOXCAR5_FILTER_LEN = 6U;

const uint16_t DC_OFFSET = 2048U;
const uint16_t RX_BLOCK_SIZE = 2U;
#ifdef TASK
const uint16_t TX_RINGBUFFER_SIZE = 10000U;
#else
const uint16_t TX_RINGBUFFER_SIZE = 5000U;
#endif
const uint16_t RX_RINGBUFFER_SIZE = 1200U;

bool        p25_tx = false;
bool        p25_duplex = false;
bool        p25_pttInvert = false;

const unsigned char BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

#define WRITE_BIT(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])
//#define READ_BIT(p,i)    (p[(i)>>3] & BIT_MASK_TABLE[(i)&7])

const unsigned char TAG_HEADER = 0x00U;
const unsigned char TAG_DATA   = 0x01U;
const unsigned char TAG_LOST   = 0x02U;
const unsigned char TAG_EOT    = 0x03U;

unsigned char*        m_txLDU1   = NULL;
unsigned char*        m_txLDU2   = NULL;
unsigned int          m_txFrames = 0;
unsigned int          m_txBits   = 0;
unsigned int          m_txErrs   = 0;
unsigned char         m_lastDUID = P25_DUID_TERM;
unsigned int          m_nac      = 0x293;
unsigned int          nac        = 0x293; // temporary setting

CRingBuffer<uint16_t> p25_txBuffer(TX_RINGBUFFER_SIZE);
CRingBuffer<uint16_t> p25_rxBuffer(RX_RINGBUFFER_SIZE);

extern CRingBuffer<uint8_t> p25PacketBuffer;
extern CRingBuffer<uint8_t> p25LDUBuffer;

std::unique_ptr< int16_t[] > baseband_txbuffer; ///< Buffer for baseband audio handling.
stream_sample_t *idleBuffer;      ///< Half baseband buffer, free for processing.
streamId         outStream;       ///< Baseband output stream ID.
pathId           outPath;         ///< Baseband output path ID.
bool             reqStop = false;
bool             txing = false;
#ifdef TASK
static void *update_tx(void *arg);
#else
bool             invertPhase = false;
#endif
P25_IO::P25_IO() :
		m_started(false),
		m_boxcar5Filter(),
		m_boxcar5State(),
		m_rxLevel(128),
		m_pttInvert(false),
		m_srcId(0U),
		m_dstId(0U),
		m_nid(nac),
		m_regen(true),
		rx_started(false)
{
	::memset(m_boxcar5State, 0x00U, 30U * sizeof(q15_t));
	m_boxcar5Filter.numTaps = BOXCAR5_FILTER_LEN;
	m_boxcar5Filter.pState  = m_boxcar5State;
	m_boxcar5Filter.pCoeffs = BOXCAR5_FILTER;

	m_txLDU1 = new unsigned char[9U * 25U];
	m_txLDU2 = new unsigned char[9U * 25U];

	::memset(m_txLDU1, 0x00U, 9U * 25U);
	::memset(m_txLDU2, 0x00U, 9U * 25U);

	baseband_rxbuffer = std::make_unique< int16_t[] >(2 * 1800);
	reset();
}

P25_IO::~P25_IO()
{
	delete[] m_txLDU1;
	delete[] m_txLDU2;
}

void P25_IO::reset()
{
	p25RX.reset();
	if(state.settings.p25_rx_level <= 0)
		state.settings.p25_rx_level = 100;
	if(state.settings.p25_tx_level <= 0)
		state.settings.p25_tx_level = 160;
	m_rxLevel = state.settings.p25_rx_level;
	setTxLevel(state.settings.p25_tx_level);
    dsp_resetFilterState(&dcrState);
}

bool P25_IO::isLocked()
{
	return locked;
}

void P25_IO::terminate()
{
	// Ensure proper termination of baseband sampling
	audioPath_release(basebandPath);
	audioStream_terminate(basebandId);
}

void P25_IO::startBasebandSampling()
{
	basebandPath = audioPath_request(SOURCE_RTX, SINK_MCU, PRIO_RX);
	basebandId = audioStream_start(basebandPath, baseband_rxbuffer.get(),
			2 * 1800, P25_RX_SAMPLE_RATE,
			STREAM_INPUT | BUF_CIRC_DOUBLE);
}

void P25_IO::stopBasebandSampling()
{
	audioStream_terminate(basebandId);
	audioPath_release(basebandPath);
    dsp_resetFilterState(&dcrState);
	locked = false;
}

void P25_IO::init_tx()
{
	/*
	 * Allocate a chunk of memory to contain two complete buffers for baseband
	 * audio.
	 */

	baseband_txbuffer = std::make_unique< int16_t[] >(2 * 180);
	idleBuffer        = baseband_txbuffer.get();
	txRunning         = false;
}

void P25_IO::terminate_tx()
{
	// Terminate an ongoing stream, if present
	if (txRunning)
	{
		audioStream_terminate(outStream);
		txRunning = false;
	}

	// Always ensure that outgoing audio path is closed
	audioPath_release(outPath);

	// Deallocate memory.
	baseband_txbuffer.reset();
}

bool P25_IO::start_tx(const bool invertP)
{
	if (txRunning)
		return true;

	setTxLevel(state.settings.p25_tx_level);

	outPath = audioPath_request(SOURCE_MCU, SINK_RTX, PRIO_TX);
	if (outPath < 0)
		return false;

	outStream = audioStream_start(outPath, baseband_txbuffer.get(),
			2*180, P25_TX_SAMPLE_RATE,
			STREAM_OUTPUT | BUF_CIRC_DOUBLE);

	if (outStream < 0)
		return false;

	idleBuffer = outputStream_getIdleBuffer(outStream);

	m_rfData.reset();
	p25_txBuffer.reset();
#ifdef TASK
	startThread(invertP, update_tx);
#else
	invertPhase = invertP;
#endif
	txRunning = true;

	return true;
}

void P25_IO::stop_tx()
{
	p25_tx = false;
	if (txRunning == false)
		return;

	stopThread();
	audioStream_stop(outStream);
	txRunning  = false;
	idleBuffer = baseband_txbuffer.get();
	audioPath_release(outPath);
}

bool P25_IO::update(const bool invertPhase)
{
	// Audio path closed, nothing to do
	if (audioPath_getStatus(basebandPath) != PATH_OPEN)
		return false;

	m_rxLevel = state.settings.p25_rx_level;

	// Read samples from the ADC
	dataBlock_t baseband = inputStream_getData(basebandId);

	// Apply DC removal filter
    dsp_dcRemoval(&dcrState, baseband.data, baseband.len);

	if (baseband.data != NULL)
	{
		// Process samples
		for (size_t i = 0; i < baseband.len; i++)
		{
			uint16_t elem = static_cast< uint16_t >(baseband.data[i]);
			if (invertPhase) elem = 0 - elem;
			p25_rxBuffer.put(elem);
			process();
		}
	}
	locked = p25RX.m_synced;
	// if header packet
	if (p25RX.isValid() && !dataValid)
	{
		dataValid = true;
		m_rfData.reset();
	}

	return p25RX.m_synced;
}
#ifdef TASK
static void *update_tx(void *arg)
{
	static int bufSize = 0;
	static bool running = false;
	if (running) return NULL;
	running = true;

	bool invertPhase = *((bool*)arg);
	memset(idleBuffer, 0x00, 1200 * sizeof(stream_sample_t));

	while (!reqStop)
	{
		if (audioPath_getStatus(outPath) != PATH_OPEN) return NULL;

		if (p25_txBuffer.getData() > 0 && bufSize < 1200 && txing)
		{
			uint16_t tmp;
			p25_txBuffer.get(tmp);
			int16_t elem = static_cast< int16_t >(tmp);
			if (invertPhase) elem = 0 - elem;    // Invert signal phase
			idleBuffer[bufSize++] = elem;
			//				if (elem > 32000 || elem < -32000)
			//			    platform_ledOn(YELLOW);
		}
		if (bufSize == 1200 || (!txing && bufSize > 0))
		{
			// Transmission is ongoing, syncronise with stream end before proceeding
			outputStream_sync(outStream, true);
			idleBuffer = outputStream_getIdleBuffer(outStream);
			memset(idleBuffer, 0x00, 1200 * sizeof(stream_sample_t));
			bufSize = 0;
		}
	}
	running = false;
	return NULL;
}
#else
void update_tx(uint16_t length)
{
	static int bufSize;

	if (audioPath_getStatus(outPath) != PATH_OPEN) return;

	if (p25_txBuffer.getData() >= length)
	{
		for (int i=0;i<length;i++)
		{
			uint16_t tmp;
			p25_txBuffer.get(tmp);
			int16_t elem = static_cast< int16_t >(tmp);
			if (invertPhase) elem = 0 - elem;    // Invert signal phase
			idleBuffer[bufSize++] = elem;
			if (bufSize == 180)
			{
				// Transmission is ongoing, syncronise with stream end before proceeding
				outputStream_sync(outStream, true);
				idleBuffer = outputStream_getIdleBuffer(outStream);
				memset(idleBuffer, 0x00, 180 * sizeof(stream_sample_t));
				bufSize = 0;
			}
		}
	}
	if (bufSize > 0)
	{
		// Transmission is ongoing, syncronise with stream end before proceeding
		outputStream_sync(outStream, true);
		idleBuffer = outputStream_getIdleBuffer(outStream);
		memset(idleBuffer, 0x00, 180 * sizeof(stream_sample_t));
		bufSize = 0;
	}
}
#endif
void P25_IO::start()
{
	if (m_started)
		return;

	m_started = true;

	p25RX.reset();

	m_rxLevel = state.settings.p25_rx_level;
	setTxLevel(state.settings.p25_tx_level);
}

bool P25_IO::isValid()
{
	return p25RX.isValid();
}

unsigned char P25_IO::decodeNID(unsigned char *data)
{
	// Decode the NID
	bool valid = m_nid.decode(data);

	unsigned char duid = m_nid.getDUID();
	if (!valid)
	{
		switch (m_lastDUID)
		{
		case P25_DUID_HEADER:
		case P25_DUID_LDU2:
			duid = P25_DUID_LDU1;
			break;
		case P25_DUID_LDU1:
			duid = P25_DUID_LDU2;
			break;
		default:
			break;
		}
	}
	return duid;
}

void P25_IO::decode()
{
	if (p25LDUBuffer.getData() < P25_LDU_FRAME_LENGTH_BYTES + 1U)
		return;

	unsigned char buffer[P25_LDU_FRAME_LENGTH_BYTES];
	unsigned char buf[1];
	p25LDUBuffer.get(buf[0]); // remove packet type byte
//	vcom_writeBlock((void*)buf, 1);
	for (int i=0;i<(int)P25_LDU_FRAME_LENGTH_BYTES;i++)
	{
		p25LDUBuffer.get(buffer[i]);
	}
	if (buf[0] == 0x01)
	    decodeLDUAudio(buffer);
}

void P25_IO::decodeLDUAudio(unsigned char *buffer)
{
//	if(!rx_started)
	//    m_rfData.reset();
	m_lastDUID = decodeNID(buffer);
	if (m_lastDUID <= 0) return;

	if (m_lastDUID == P25_DUID_LDU1)// && !rx_started)
	{
		m_rfData.decodeLDU1(buffer);
		if (m_rfData.getSrcId() > 0)
		{
			m_srcId = m_rfData.getSrcId();
			m_dstId = m_rfData.getDstId();
			rx_started = true;
		}
	}

	if (m_lastDUID == P25_DUID_LDU2)
	    m_rfData.decodeLDU2(buffer);

	m_audio.process(buffer);
	unsigned char imbe[11] = {0x04U, 0x0CU, 0xFDU, 0x7BU, 0xFBU, 0x7DU, 0xF2U, 0x7BU, 0x3DU, 0x9EU, 0x45U};

	m_audio.decode(buffer, imbe, 0U);
	for (int i=0;i<11;i++)
		p25PacketBuffer.put(imbe[i]);
	m_audio.decode(buffer, imbe, 1U);
	for (int i=0;i<11;i++)
		p25PacketBuffer.put(imbe[i]);
	m_audio.decode(buffer, imbe, 2U);
	for (int i=0;i<11;i++)
		p25PacketBuffer.put(imbe[i]);
	m_audio.decode(buffer, imbe, 3U);
	for (int i=0;i<11;i++)
		p25PacketBuffer.put(imbe[i]);
	m_audio.decode(buffer, imbe, 4U);
	for (int i=0;i<11;i++)
		p25PacketBuffer.put(imbe[i]);
	m_audio.decode(buffer, imbe, 5U);
	for (int i=0;i<11;i++)
		p25PacketBuffer.put(imbe[i]);
	m_audio.decode(buffer, imbe, 6U);
	for (int i=0;i<11;i++)
		p25PacketBuffer.put(imbe[i]);
	m_audio.decode(buffer, imbe, 7U);
	for (int i=0;i<11;i++)
		p25PacketBuffer.put(imbe[i]);
	m_audio.decode(buffer, imbe, 8U);
	for (int i=0;i<11;i++)
		p25PacketBuffer.put(imbe[i]);
}

void P25_IO::addBusyBits(unsigned char* data, unsigned int length, bool b1, bool b2)
{
	for (unsigned int ss0Pos = P25_SS0_START; ss0Pos < length; ss0Pos += P25_SS_INCREMENT) {
		unsigned int ss1Pos = ss0Pos + 1U;
		WRITE_BIT(data, ss0Pos, b1);
		WRITE_BIT(data, ss1Pos, b2);
	}
}

void P25_IO::addP25Sync(unsigned char* data)
{
	::memcpy(data, P25_SYNC_BYTES, P25_SYNC_LENGTH_BYTES);
}

void P25_IO::writeSyncFrames()
{
	p25TX.writeSyncFrames();
	m_regen = true;
}

void P25_IO::createTxHeader()
{
	static unsigned char buffer[P25_HDR_FRAME_LENGTH_BYTES];
	static uint32_t lastSrcId;
	static uint32_t lastDstId;

	rx_started = false;

//	if (lastSrcId != m_rfData.getSrcId() || lastDstId != m_rfData.getDstId())
	if (m_regen)
	{
		::memset(buffer, 0x00U, P25_HDR_FRAME_LENGTH_BYTES);

	//	lastSrcId = m_rfData.getSrcId();
	//	lastDstId = m_rfData.getDstId();
	//	m_regen = true;

		// Add the sync
		addP25Sync(buffer);

		//	m_rfData.reset();
		//	m_rfData.setMI(mi);
		//	m_rfData.setAlgId(algId);
		//	m_rfData.setKId(kId);
		if (m_rfData.getDstId() > 0 && m_rfData.getDstId() <= 65535)
			m_rfData.setLCF(0);
		else
			m_rfData.setLCF(3);
		//	m_rfData.setMFId(mfId);

		// Add the NID
		m_nid.encode(buffer, P25_DUID_HEADER);

		// Add the dummy header
		m_rfData.encodeHeader(buffer);

		// Add busy bits
		addBusyBits(buffer, P25_HDR_FRAME_LENGTH_BITS, true, false);

		m_lastDUID = P25_DUID_HEADER;
	}
	setTxData(buffer, P25_HDR_FRAME_LENGTH_BYTES);
}

void P25_IO::createTxLDU1()
{
	static unsigned char buffer[P25_LDU_FRAME_LENGTH_BYTES];

	if (m_regen)
	{
		::memset(buffer, 0x00U, P25_LDU_FRAME_LENGTH_BYTES);

		// Add the sync
		addP25Sync(buffer);

		// Add the NID
		m_nid.encode(buffer, P25_DUID_LDU1);

		// Add the LDU1 data
		m_rfData.encodeLDU1(buffer);
	}
	// Add the Audio
	unsigned char imbe[11] = {0x04U, 0x0CU, 0xFDU, 0x7BU, 0xFBU, 0x7DU, 0xF2U, 0x7BU, 0x3DU, 0x9EU, 0x45U}; // silence
	if (p25PacketBuffer.getData() >= 99)
	{
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 0U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 1U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 2U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 3U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 4U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 5U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 6U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 7U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 8U);
	}
	else
	{   // No audio data add silence
		m_audio.encode(buffer, imbe, 0U);
		m_audio.encode(buffer, imbe, 1U);
		m_audio.encode(buffer, imbe, 2U);
		m_audio.encode(buffer, imbe, 3U);
		m_audio.encode(buffer, imbe, 4U);
		m_audio.encode(buffer, imbe, 5U);
		m_audio.encode(buffer, imbe, 6U);
		m_audio.encode(buffer, imbe, 7U);
		m_audio.encode(buffer, imbe, 8U);
	}

	if (m_regen)
	{
		// Add the Low Speed Data
		m_txLSD.setLSD1(m_txLDU1[201U]);
		m_txLSD.setLSD2(m_txLDU1[202U]);
		m_txLSD.encode(buffer);
	}
	// Add busy bits
	addBusyBits(buffer, P25_LDU_FRAME_LENGTH_BITS, true, false);

	setTxData(buffer, P25_LDU_FRAME_LENGTH_BYTES);
	//	sleepFor(0, 150);
}

void P25_IO::createTxLDU2()
{
	static unsigned char buffer[P25_LDU_FRAME_LENGTH_BYTES];

	if (m_regen)
	{
		::memset(buffer, 0x00U, P25_LDU_FRAME_LENGTH_BYTES);

		// Add the sync
		addP25Sync(buffer);

		// Add the NID
		m_nid.encode(buffer, P25_DUID_LDU2);

		// Add the dummy LDU2 data
		m_rfData.encodeLDU2(buffer);
	}
	// Add the Audio
	unsigned char imbe[11] = {0x04U, 0x0CU, 0xFDU, 0x7BU, 0xFBU, 0x7DU, 0xF2U, 0x7BU, 0x3DU, 0x9EU, 0x45U}; // silence
	if (p25PacketBuffer.getData() >= 99)
	{
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 0U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 1U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 2U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 3U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 4U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 5U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 6U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 7U);
		for (int i=0;i<11;i++)
			p25PacketBuffer.get(imbe[i]);
		m_audio.encode(buffer, imbe, 8U);
	}
	else
	{  // No audio data add silence
		m_audio.encode(buffer, imbe, 0U);
		m_audio.encode(buffer, imbe, 1U);
		m_audio.encode(buffer, imbe, 2U);
		m_audio.encode(buffer, imbe, 3U);
		m_audio.encode(buffer, imbe, 4U);
		m_audio.encode(buffer, imbe, 5U);
		m_audio.encode(buffer, imbe, 6U);
		m_audio.encode(buffer, imbe, 7U);
		m_audio.encode(buffer, imbe, 8U);
	}

	if (m_regen)
	{
		// Add the Low Speed Data
		m_txLSD.setLSD1(m_txLDU2[201U]);
		m_txLSD.setLSD2(m_txLDU2[202U]);
		m_txLSD.encode(buffer);

		m_regen = false;
	}
	// Add busy bits
	addBusyBits(buffer, P25_LDU_FRAME_LENGTH_BITS, true, false);

	setTxData(buffer, P25_LDU_FRAME_LENGTH_BYTES);
}

void P25_IO::createTxTerminator()
{
	unsigned char buffer[P25_TERM_FRAME_LENGTH_BYTES];
	::memset(buffer, 0x00U, P25_TERM_FRAME_LENGTH_BYTES);

	// Add the sync
	addP25Sync(buffer);

	// Add the NID
	m_nid.encode(buffer, P25_DUID_TERM);

	// Add busy bits
	addBusyBits(buffer, P25_TERM_FRAME_LENGTH_BITS, true, false);

	setTxData(buffer, P25_TERM_FRAME_LENGTH_BYTES);

	m_rfData.reset();
}

void P25_IO::process()
{
	if (p25_rxBuffer.getData() >= RX_BLOCK_SIZE)
	{
		q15_t samples[RX_BLOCK_SIZE + 1];

		for (uint16_t i = 0; i < RX_BLOCK_SIZE; i++)
		{
			uint16_t sample;
			p25_rxBuffer.get(sample);
			q15_t res1 = q15_t(sample);// - DC_OFFSET;
			q31_t res2 = res1 * (m_rxLevel * 128);
			samples[i] = q15_t(__SSAT((res2 >> 15), 16));
		}

		q15_t P25Vals[RX_BLOCK_SIZE];
		::arm_fir_fast_q15(&m_boxcar5Filter, samples, P25Vals, RX_BLOCK_SIZE);
		p25RX.samples(P25Vals, &m_rssi, RX_BLOCK_SIZE);
	}
}

bool P25_IO::startThread(const bool invertP, void *(*func) (void *))
{
	reqStop     = false;
	bool invert = invertP;

	pthread_attr_init(&dacAttr);

#if defined(_MIOSIX)
	// Set stack size of IMBE thread to 16kB.
	pthread_attr_setstacksize(&dacAttr, 10000);

	// Set priority of IMBE thread to the maximum one, the same of RTX thread.
	struct sched_param param;
	param.sched_priority = THREAD_PRIO_HIGH;
	pthread_attr_setschedparam(&dacAttr, &param);
#elif defined(__ZEPHYR__)
	// Allocate and set the stack for AMBE thread
	void *ambe_thread_stack = malloc(10000 * sizeof(uint8_t));
	pthread_attr_setstack(&dacAttr, dac_thread_stack, 10000);
#endif

	// Start thread
	int ret = pthread_create(&dacThread, &dacAttr, func, (void*)&invert);
	if (ret < 0)
		dac_running = false;

	return dac_running;
}

void P25_IO::stopThread()
{
	reqStop = true;
	pthread_join(dacThread, NULL);
	dac_running = false;

#ifdef __ZEPHYR__
	void  *addr;
	size_t size;

	pthread_attr_getstack(&dacAttr, &addr, &size);
	free(addr);
#endif
}

unsigned int P25_IO::getSrcId()
{
	return m_srcId;
}

unsigned int P25_IO::getDstId()
{
	return m_dstId;
}

void P25_IO::setSrcId(unsigned int id)
{
	m_rfData.setSrcId(id);
}

void P25_IO::setDstId(unsigned int id)
{
	m_rfData.setDstId(id);
}

void P25_IO::setTxData(uint8_t *data, uint8_t len)
{
	p25TX.writeData(data, len);
}

void P25_IO::process_tx()
{
	p25TX.process();
}

void P25_IO::setTxLevel(uint8_t level)
{
	p25TX.setTxLevel(level);
}

void P25_IO::setTXDelay(uint8_t delay)
{
	p25TX.setTXDelay(delay);
}

void P25_IO::getData(uint8_t *data)
{
	uint8_t d[9];

	p25RX.getData(d);
	::memcpy(data, d, 9);
}

//#endif // if P25
