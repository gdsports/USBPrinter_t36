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
 * MIT License
 * 
 * Copyright (c) 2018 gdsports625@gmail.com
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/* USB printer driver
 * This driver is based on serial.cpp from the PJRC Teensy 3.6
 * USBHost_t36 library. The modifications for USB printers are 
 * by gdsports625@gmail.com
 *
 * Reference: http://www.usb.org/developers/docs/devclass_docs/usbprint11a021811.pdf
 *
 */


#include <Arduino.h>
#include "USBHost_t36.h"  // Read this header first for key info
#include "USBPrinter_t36.h"

#define print   USBHost::print_
#define println USBHost::println_

/************************************************************/
//  Initialization and claiming of devices & interfaces
/************************************************************/

void USBPrinter::init()
{
	contribute_Pipes(mypipes, sizeof(mypipes)/sizeof(Pipe_t));
	contribute_Transfers(mytransfers, sizeof(mytransfers)/sizeof(Transfer_t));
	contribute_String_Buffers(mystring_bufs, sizeof(mystring_bufs)/sizeof(strbuf_t));
	driver_ready_for_device(this);
}

bool USBPrinter::claim(Device_t *dev, int type, const uint8_t *descriptors, uint32_t len)
{
	// only claim at interface level
	println("USBPrinter claim this=", (uint32_t)this, HEX);
	print("vid=", dev->idVendor, HEX);
	print(", pid=", dev->idProduct, HEX);
	print(", bDeviceClass = ", dev->bDeviceClass);
	print(", bDeviceSubClass = ", dev->bDeviceSubClass);
	print(", bDeviceProtocol = ", dev->bDeviceProtocol);
	println(", type =", type);
	print_hexbytes(descriptors, len);
	//---------------------------------------------------------------------
	// It is a printer device see if we can extract the data...
	// But first pass see if we can simply look at the interface...
	// Lets walk through end points and see if we
	// can find an RX and TX bulk transfer end point.
	const uint8_t *p = descriptors;
	const uint8_t *end = p + len;

	if (p[0] != 9 || p[1] != 4) return false; // interface descriptor
	//println("  bInterfaceClass=", p[5]);
	//println("  bInterfaceSubClass=", p[6]);
	//println("  bInterfaceProtocol=", p[7]);
	if (p[5] != 7) return false; // bInterfaceClass: 7 Printer
	if (p[6] != 1) return false; // bInterfaceSubClass: 1 Printers
	if (p[7] != 2) return false; // bInterfaceProtocol: 2 Bi-directional interface
	interface = p[2];
	println("    Interface: ", interface);
	alternate = p[3];
	println("    Alternate: ", alternate);
	p += 9;
	println("  Interface is Bi-directional printer");
	uint8_t rx_ep = 0;
	uint8_t tx_ep = 0;
	uint16_t rx_size = 0;
	uint16_t tx_size = 0;

	while (p < end) {
		len = *p;
		print_hexbytes(p, len);
		if (len < 4) return false;
		if (p + len > end) return false; // reject if beyond end of data
		uint32_t type = p[1];
		println("type: ", type);
		if (type == 4 ) {
			break;
		}
		else if (type == 5) {
			// endpoint descriptor
			if (p[0] < 7) return false; // at least 7 bytes
			if (p[3] == 2) {  // First try ignore the first one which is interrupt...
				println("     Endpoint: ", p[2], HEX);
				switch (p[2] & 0xF0) {
					case 0x80:
						// IN endpoint
						if (rx_ep == 0) {
							rx_ep = p[2] & 0x0F;
							rx_size = p[4] | (p[5] << 8);
							println("      rx_size = ", rx_size);
						}
						break;
					case 0x00:
						// OUT endpoint
						if (tx_ep == 0) {
							tx_ep = p[2];
							tx_size = p[4] | (p[5] << 8);
							println("      tx_size = ", tx_size);
						}
						break;
					default:
						println("  invalid end point: ", p[2]);
						return false;
				}
			}
		} else {
			println("  Unknown type: ", type);
			return false; // unknown
		}
		p += len;
	}
	print("  exited loop rx:", rx_ep);
	println(", tx:", tx_ep);
	if (!rx_ep || !tx_ep) return false; 	// did not get our two end points
	if (!init_buffers(rx_size, tx_size)) return false;
	println("  rx buffer size:", rxsize);
	println("  tx buffer size:", txsize);
	rxpipe = new_Pipe(dev, 2, rx_ep & 15, 1, rx_size);
	if (!rxpipe) return false;
	txpipe = new_Pipe(dev, 2, tx_ep, 0, tx_size);
	if (!txpipe) {
		// TODO: free rxpipe
		return false;
	}
	rxpipe->callback_function = rx_callback;
	queue_Data_Transfer(rxpipe, rx1, (rx_size < 64)? rx_size : 64, this);
	rxstate = 1;
	if (rx_size > 128) {
		queue_Data_Transfer(rxpipe, rx2, rx_size, this);
		rxstate = 3;
	}
	txstate = 0;
	txpipe->callback_function = tx_callback;
	// Wish I could just call Control to do the output... Maybe can defer until the user calls begin()
	// control requires that device is setup which is not until this call completes...
	println("Control ");
	mk_setup(setalternate, 0x01, 0x0b, alternate, interface, 0);
	queue_Control_Transfer(dev, &setalternate, NULL, this);
	// GET_DEVICE_ID
	mk_setup(setup, 0xA1, 0, 0, 1, 0x3FF);
	queue_Control_Transfer(dev, &setup, NULL, this);
	control_queued = true;
	pending_control = 0x0;	// Maybe don't need to do...
	return true;
}

// check if two legal endpoints, 1 receive & 1 transmit
bool USBPrinter::check_rxtx_ep(uint32_t &rxep, uint32_t &txep)
{
	if ((rxep & 0x0F) == 0) return false;
	if ((txep & 0x0F) == 0) return false;
	uint32_t rxdir = rxep & 0xF0;
	uint32_t txdir = txep & 0xF0;
	if (rxdir == 0x80 && txdir == 0x00) {
		return true;
	}
	if (rxdir == 0x00 && txdir == 0x80) {
		std::swap(rxep, txep);
		return true;
	}
	return false;
}

// initialize buffer sizes and pointers
bool USBPrinter::init_buffers(uint32_t rsize, uint32_t tsize)
{
	// buffer must be able to hold 2 of each packet, plus buffer
	// space to hold RX and TX data.
	if (sizeof(bigbuffer) < (rsize + tsize) * 3 + 2) return false;
	rx1 = (uint8_t *)bigbuffer;
	rx2 = rx1 + rsize;
	tx1 = rx2 + rsize;
	tx2 = tx1 + tsize;
	rxbuf = tx2 + tsize;
	// FIXME: this assume 50-50 split - not true when rsize != tsize
	rxsize = (sizeof(bigbuffer) - (rsize + tsize) * 2) / 2;
	txsize = rxsize;
	txbuf = rxbuf + rxsize;
	rxhead = 0;
	rxtail = 0;
	txhead = 0;
	txtail = 0;
	rxstate = 0;
	return true;
}

void USBPrinter::disconnect()
{
}

/************************************************************/
//  Interrupt-based Data Movement
/************************************************************/

void USBPrinter::rx_callback(const Transfer_t *transfer)
{
	if (!transfer->driver) return;
	((USBPrinter *)(transfer->driver))->rx_data(transfer);
}

void USBPrinter::tx_callback(const Transfer_t *transfer)
{
	if (!transfer->driver) return;
	((USBPrinter *)(transfer->driver))->tx_data(transfer);
}


void USBPrinter::rx_data(const Transfer_t *transfer)
{
	uint32_t len = transfer->length - ((transfer->qtd.token >> 16) & 0x7FFF);

	// first update rxstate bitmask, since buffer is no longer queued
	if (transfer->buffer == rx1) {
		rxstate &= 0xFE;
	} else if (transfer->buffer == rx2) {
		rxstate &= 0xFD;
	}
	// get start of data and actual length
	const uint8_t *p = (const uint8_t *)transfer->buffer;
	if (len > 0) {
		print("rx token: ", transfer->qtd.token, HEX);
		print(" transfer length: ", transfer->length, DEC);
		print(" len:", len, DEC);
		print(" - ", *p, HEX);
		println(" ", *(p+1), HEX);
		print("rx: ");
		print_hexbytes(p, len);
	}
	// Copy data from packet buffer to circular buffer.
	// Assume the buffer will always have space, since we
	// check before queuing the buffers
	uint32_t head = rxhead;
	uint32_t tail = rxtail;
	if (++head >= rxsize) head = 0;
	uint32_t avail;
	if (len > 0) {
		//print("head=", head);
		//print(", tail=", tail);
		avail = rxsize - head;
		//print(", avail=", avail);
		//println(", rxsize=", rxsize);
		if (avail > len) avail = len;
		memcpy(rxbuf + head, p, avail);
		if (len <= avail) {
			head += avail - 1;
			if (head >= rxsize) head = 0;
		} else {
			head = len - avail - 1;
			memcpy(rxbuf, p + avail, head + 1);
		}
		rxhead = head;
	}
	// TODO: can be this more efficient?  We know from above which
	// buffer is no longer queued, so possible skip most of this work?
	rx_queue_packets(head, tail);
}

// re-queue packet buffer(s) if possible
void USBPrinter::rx_queue_packets(uint32_t head, uint32_t tail)
{
	uint32_t avail;
	if (head >= tail) {
		avail = rxsize - 1 - head + tail;
	} else {
		avail = tail - head - 1;
	}
	uint32_t packetsize = rx2 - rx1;
	if (avail >= packetsize) {
		if ((rxstate & 0x01) == 0) {
			queue_Data_Transfer(rxpipe, rx1, packetsize, this);
			rxstate |= 0x01;
		} else if ((rxstate & 0x02) == 0) {
			queue_Data_Transfer(rxpipe, rx2, packetsize, this);
			rxstate |= 0x02;
		}
		if ((rxstate & 0x03) != 0x03 && avail >= packetsize * 2) {
			if ((rxstate & 0x01) == 0) {
				queue_Data_Transfer(rxpipe, rx1, packetsize, this);
				rxstate |= 0x01;
			} else if ((rxstate & 0x02) == 0) {
				queue_Data_Transfer(rxpipe, rx2, packetsize, this);
				rxstate |= 0x02;
			}
		}
	}
}

void USBPrinter::tx_data(const Transfer_t *transfer)
{
	uint32_t mask;
	uint8_t *p = (uint8_t *)transfer->buffer;
	if (p == tx1) {
		println("tx1:");
		mask = 1;
		//txstate &= 0xFE;
	} else if (p == tx2) {
		println("tx2:");
		mask = 2;
		//txstate &= 0xFD;
	} else {
		return; // should never happen
	}
	// check how much more data remains in the transmit buffer
	uint32_t head = txhead;
	uint32_t tail = txtail;
	uint32_t count;
	if (head >= tail) {
		count = head - tail;
	} else {
		count = txsize + head - tail;
	}
	uint32_t packetsize = tx2 - tx1;
	// Only output full packets unless the flush bit was set.
	if ((count == 0) || ((count < packetsize) && ((txstate & 0x4) == 0) )) {
		// not enough data in buffer to fill a full packet
		txstate &= ~(mask | 4);	// turn off that transfer and make sure the flush bit is not set
		return;
	}
	// immediately transmit another full packet, if we have enough data
	if (count >= packetsize) count = packetsize;
	else txstate &= ~(mask | 4); // This packet will complete any outstanding flush

	println("TX:moar data!!!!");
	if (++tail >= txsize) tail = 0;
	uint32_t n = txsize - tail;
	if (n > count) n = count;
	memcpy(p, txbuf + tail, n);
	if (n >= count) {
		tail += n - 1;
		if (tail >= txsize) tail = 0;
	} else {
		uint32_t len = count - n;
		memcpy(p + n, txbuf, len);
		tail = len - 1;
	}
	txtail = tail;
	queue_Data_Transfer(txpipe, p, count, this);
}

void USBPrinter::flush()
{
	print("USBPrinter::flush");
	if (txhead == txtail) {
		println(" - Empty");
		return;  // empty.
	}
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	txtimer.stop();  		// Stop longer timer.
	txtimer.start(100);		// Start a mimimal timeout
//	timer_event(nullptr);   // Try calling direct - fails to work
	NVIC_ENABLE_IRQ(IRQ_USBHS);
	while (txstate & 3) ; // wait for all of the USB packets to be sent.
	println(" completed");
}



void USBPrinter::timer_event(USBDriverTimer *whichTimer)
{
	println("txtimer");
	uint32_t count;
	uint32_t head = txhead;
	uint32_t tail = txtail;
	if (head == tail) {
		println("  *** Empty ***");
		return; // nothing to transmit
	} else if (head > tail) {
		count = head - tail;
	} else {
		count = txsize + head - tail;
	}

	uint8_t *p;
	if ((txstate & 0x01) == 0) {
		p = tx1;
		txstate |= 0x01;
	} else if ((txstate & 0x02) == 0) {
		p = tx2;
		txstate |= 0x02;
	} else {
		txstate |= 4; 	// Tell the TX code to do flush code.
		println(" *** No buffers ***");
		return; // no outgoing buffers available, try again later
	}

	uint32_t packetsize = tx2 - tx1;

	// Possible for remaining ? packet size and not have both?
	if (count > packetsize) {
		txstate |= 4;	// One of the active transfers will handle the remaining parts
		count = packetsize;
	}

	if (++tail >= txsize) tail = 0;
	uint32_t n = txsize - tail;
	if (n > count) n = count;
	memcpy(p, txbuf + tail, n);
	if (n >= count) {
		tail += n - 1;
		if (tail >= txsize) tail = 0;
	} else {
		uint32_t len = count - n;
		memcpy(p + n, txbuf, len);
		tail = len - 1;
	}
	txtail = tail;
	print("  TX data (", count);
	print(") ");
	print_hexbytes(p, count);
	queue_Data_Transfer(txpipe, p, count, this);
}



/************************************************************/
//  User Functions - must disable USBHQ IRQ for EHCI access
/************************************************************/

void USBPrinter::begin()
{
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	if (!control_queued) control(NULL);
	NVIC_ENABLE_IRQ(IRQ_USBHS);
	// Wait until all packets have been queued before we return to caller.
	while (pending_control) {
		yield();	// not sure if we want to yield or what?
	}
}

void USBPrinter::end(void)
{
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	if (!control_queued) control(NULL);
	NVIC_ENABLE_IRQ(IRQ_USBHS);

	// Wait until all packets have been queued before we return to caller.
	while (pending_control) {
		yield();	// not sure if we want to yield or what?
	}
}

int USBPrinter::available(void)
{
	if (!device) return 0;
	uint32_t head = rxhead;
	uint32_t tail = rxtail;
	if (head >= tail) return head - tail;
	return rxsize + head - tail;
}

int USBPrinter::peek(void)
{
	if (!device) return -1;
	uint32_t head = rxhead;
	uint32_t tail = rxtail;
	if (head == tail) return -1;
	if (++tail >= rxsize) tail = 0;
	return rxbuf[tail];
}

int USBPrinter::read(void)
{
	if (!device) return -1;
	uint32_t head = rxhead;
	uint32_t tail = rxtail;
	if (head == tail) return -1;
	if (++tail >= rxsize) tail = 0;
	int c = rxbuf[tail];
	rxtail = tail;
	if ((rxstate & 0x03) != 0x03) {
		NVIC_DISABLE_IRQ(IRQ_USBHS);
		rx_queue_packets(head, tail);
		NVIC_ENABLE_IRQ(IRQ_USBHS);
	}
	return c;
}

int USBPrinter::availableForWrite()
{
	if (!device) return 0;
	uint32_t head = txhead;
	uint32_t tail = txtail;
	if (head >= tail) return txsize - 1 - head + tail;
	return tail - head - 1;
}

size_t USBPrinter::write(uint8_t c)
{
	if (!device) return 0;
	uint32_t head = txhead;
	if (++head >= txsize) head = 0;
	while (txtail == head) {
		// wait...
	}
	txbuf[head] = c;
	txhead = head;
	//print("head=", head);
	//println(", tail=", txtail);

	// if full packet in buffer and tx packet ready, queue it
	NVIC_DISABLE_IRQ(IRQ_USBHS);
	uint32_t tail = txtail;
	if ((txstate & 0x03) != 0x03) {
		// at least one packet buffer is ready to transmit
		uint32_t count;
		if (head >= tail) {
			count = head - tail;
		} else {
			count = txsize + head - tail;
		}
		uint32_t packetsize = tx2 - tx1;
		if (count >= packetsize) {
			//println("txsize=", txsize);
			uint8_t *p;
			if ((txstate & 0x01) == 0) {
				p = tx1;
				txstate |= 0x01;
			} else /* if ((txstate & 0x02) == 0) */ {
				p = tx2;
				txstate |= 0x02;
			}
			// copy data to packet buffer
			if (++tail >= txsize) tail = 0;
			uint32_t n = txsize - tail;
			if (n > packetsize) n = packetsize;
			//print("memcpy, offset=", tail);
			//println(", len=", n);
			memcpy(p, txbuf + tail, n);
			if (n >= packetsize) {
				tail += n - 1;
				if (tail >= txsize) tail = 0;
			} else {
				//n = txsize - n;
				uint32_t len = packetsize - n;
				//println("memcpy, offset=0, len=", len);
				memcpy(p + n, txbuf, len);
				tail = len - 1;
			}
			txtail = tail;
			//println("queue tx packet, newtail=", tail);
			queue_Data_Transfer(txpipe, p, packetsize, this);
			NVIC_ENABLE_IRQ(IRQ_USBHS);
			return 1;
		}
	}
	// otherwise, set a latency timer to later transmit partial packet
	txtimer.stop();
	txtimer.start(write_timeout_);
	NVIC_ENABLE_IRQ(IRQ_USBHS);
	return 1;
}
