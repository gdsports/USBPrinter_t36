/* USB EHCI Host for Teensy 3.6
 * Copyright 2017 Paul Stoffregen (paul@pjrc.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/*
MIT License

Copyright (c) 2018 gdsports625@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/* USB printer driver
 * This driver is based on serial.cpp from the PJRC Teensy 3.6
 * USBHost_t36 library. The modifications for USB printers are 
 * made by gdsports625@gmail.com
 *
 * Reference: http://www.usb.org/developers/docs/devclass_docs/usbprint11a021811.pdf
 *
 */

class USBPrinter: public USBDriver, public Stream {
	public:


	enum { BUFFER_SIZE = 648 }; // must hold at least 6 max size packets, plus 2 extra bytes
	enum { DEFAULT_WRITE_TIMEOUT = 3500};
	USBPrinter(USBHost &host) : txtimer(this) { init(); }
	void begin();
	void end(void);
	uint32_t writeTimeout() {return write_timeout_;}
	void writeTimeOut(uint32_t write_timeout) {write_timeout_ = write_timeout;} // Will not impact current ones.
	virtual int available(void);
	virtual int peek(void);
	virtual int read(void);
	virtual int availableForWrite();
	virtual size_t write(uint8_t c);
	virtual void flush(void);

	using Print::write;
protected:
	virtual bool claim(Device_t *device, int type, const uint8_t *descriptors, uint32_t len);
	virtual void disconnect();
	virtual void timer_event(USBDriverTimer *whichTimer);
private:
	static void rx_callback(const Transfer_t *transfer);
	static void tx_callback(const Transfer_t *transfer);
	void rx_data(const Transfer_t *transfer);
	void tx_data(const Transfer_t *transfer);
	void rx_queue_packets(uint32_t head, uint32_t tail);
	void init();
	static bool check_rxtx_ep(uint32_t &rxep, uint32_t &txep);
	bool init_buffers(uint32_t rsize, uint32_t tsize);
	void ch341_setBaud(uint8_t byte_index);
private:
	Pipe_t mypipes[3] __attribute__ ((aligned(32)));
	Transfer_t mytransfers[7] __attribute__ ((aligned(32)));
	strbuf_t mystring_bufs[1];
	USBDriverTimer txtimer;
	uint32_t bigbuffer[(BUFFER_SIZE+3)/4];
	setup_t setup;
	setup_t setalternate;
	uint8_t setupdata[16]; //
	uint32_t write_timeout_ = DEFAULT_WRITE_TIMEOUT;
	Pipe_t *rxpipe;
	Pipe_t *txpipe;
	uint8_t *rx1;	// location for first incoming packet
	uint8_t *rx2;	// location for second incoming packet
	uint8_t *rxbuf;	// receive circular buffer
	uint8_t *tx1;	// location for first outgoing packet
	uint8_t *tx2;	// location for second outgoing packet
	uint8_t *txbuf;
	volatile uint16_t rxhead;// receive head
	volatile uint16_t rxtail;// receive tail
	volatile uint16_t txhead;
	volatile uint16_t txtail;
	uint16_t rxsize;// size of receive circular buffer
	uint16_t txsize;// size of transmit circular buffer
	volatile uint8_t  rxstate;// bitmask: which receive packets are queued
	volatile uint8_t  txstate;
	uint8_t pending_control;
	uint8_t interface;
	uint8_t alternate;
	bool control_queued;
};

