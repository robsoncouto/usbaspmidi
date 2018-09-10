/* Name: main.c
 * MIDI CONVERTER
 *
 * MOCO: Midi Output Converter
 * MICO: Midi Input Converter
 *
 * (C) 2010 by morecat_lab
 */


#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "usbdrv.h"
#include "oddebug.h"

extern void parseUSBMidiMessage(uchar *, uchar);
extern uchar parseSerialMidiMessage(uchar);

#define setPortBit0(port, bit) *(port) &= ~_BV(bit);
#define setPortBit1(port, bit) *(port) |= _BV(bit);
#define flipPortBit(port, bit) (*(port) ^= _BV(bit))
#define setPortBit(port, bit, val) { \
    if ((val) == 0) { setPortBit0((port), (bit)); } else	\
      { setPortBit1((port), (bit)); }				\
}

/* debugging LED */
// #ifdef ARDUINO
// #define LED_ON  { PORTB &= ~0x20; }
// #define LED_OFF   { PORTB |=  0x20; }
// #define FLIP_LED { PORTB ^=  0x20; }
// #else
#define LED_ON  { PORTC |=  0x01; }
#define LED_OFF   { PORTC &= ~0x01; }
#define FLIP_LED { PORTC ^=  0x01; }
// #endif

// #define RECEIVER
// #define TRANSMITTER

/*
 PORT ASSIGN
 PD0: RxD (Serial MIDI)
 PD1: TxD (Serial MIDI)
 PD2: USB D+
 PD3: USB D-

 PD5: LED Monitor

 */

#define HW_CDC_BULK_OUT_SIZE     8
#define HW_CDC_BULK_IN_SIZE      8

#define	TRUE			1
#define	FALSE			0

enum {
    SEND_ENCAPSULATED_COMMAND = 0,
    GET_ENCAPSULATED_RESPONSE,
    SET_COMM_FEATURE,
    GET_COMM_FEATURE,
    CLEAR_COMM_FEATURE,
    SET_LINE_CODING = 0x20,
    GET_LINE_CODING,
    SET_CONTROL_LINE_STATE,
    SEND_BREAK
};

// This deviceDescrMIDI[] is based on 
// http://www.usb.org/developers/devclass_docs/midi10.pdf
// Appendix B. Example: Simple MIDI Adapter (Informative)

// B.1 Device Descriptor
const PROGMEM char deviceDescrMIDI[] = {	/* USB device descriptor */
  18,			/* sizeof(usbDescriptorDevice): length of descriptor in bytes */
  USBDESCR_DEVICE,	/* descriptor type */
  0x10, 0x01,		/* USB version supported */
  0,			/* device class: defined at interface level */
  0,			/* subclass */
  0,			/* protocol */
  8,			/* max packet size */
  USB_CFG_VENDOR_ID,	/* 2 bytes */
  USB_CFG_DEVICE_ID,	/* 2 bytes */
  USB_CFG_DEVICE_VERSION,	/* 2 bytes */
  1,			/* manufacturer string index */
  2,			/* product string index */
  0,			/* serial number string index */
  1,			/* number of configurations */
};

// B.2 Configuration Descriptor
const PROGMEM char configDescrMIDI[] = { /* USB configuration descriptor */
  9,	   /* sizeof(usbDescrConfig): length of descriptor in bytes */
  USBDESCR_CONFIG,		/* descriptor type */
  101, 0,			/* total length of data returned (including inlined descriptors) */
  2,		      /* number of interfaces in this configuration */
  1,				/* index of this configuration */
  0,				/* configuration name string index */
#if USB_CFG_IS_SELF_POWERED
    (1 << 7) | USBATTR_SELFPOWER,       /* attributes */
#else
    (1 << 7),                           /* attributes */
#endif
  USB_CFG_MAX_BUS_POWER / 2,	/* max USB current in 2mA units */

// B.3 AudioControl Interface Descriptors
// The AudioControl interface describes the device structure (audio function topology) 
// and is used to manipulate the Audio Controls. This device has no audio function 
// incorporated. However, the AudioControl interface is mandatory and therefore both 
// the standard AC interface descriptor and the classspecific AC interface descriptor 
// must be present. The class-specific AC interface descriptor only contains the header 
// descriptor.

// B.3.1 Standard AC Interface Descriptor
// The AudioControl interface has no dedicated endpoints associated with it. It uses the 
// default pipe (endpoint 0) for all communication purposes. Class-specific AudioControl 
// Requests are sent using the default pipe. There is no Status Interrupt endpoint provided.
  /* descriptor follows inline: */
  9,			/* sizeof(usbDescrInterface): length of descriptor in bytes */
  USBDESCR_INTERFACE,	/* descriptor type */
  0,			/* index of this interface */
  0,			/* alternate setting for this interface */
  0,			/* endpoints excl 0: number of endpoint descriptors to follow */
  1,			/* */
  1,			/* */
  0,			/* */
  0,			/* string index for interface */

// B.3.2 Class-specific AC Interface Descriptor
// The Class-specific AC interface descriptor is always headed by a Header descriptor 
// that contains general information about the AudioControl interface. It contains all 
// the pointers needed to describe the Audio Interface Collection, associated with the 
// described audio function. Only the Header descriptor is present in this device 
// because it does not contain any audio functionality as such.
  /* descriptor follows inline: */
  9,			/* sizeof(usbDescrCDC_HeaderFn): length of descriptor in bytes */
  36,			/* descriptor type */
  1,			/* header functional descriptor */
  0x0, 0x01,		/* bcdADC */
  9, 0,			/* wTotalLength */
  1,			/* */
  1,			/* */

// B.4 MIDIStreaming Interface Descriptors

// B.4.1 Standard MS Interface Descriptor
  /* descriptor follows inline: */
  9,			/* length of descriptor in bytes */
  USBDESCR_INTERFACE,	/* descriptor type */
  1,			/* index of this interface */
  0,			/* alternate setting for this interface */
  2,			/* endpoints excl 0: number of endpoint descriptors to follow */
  1,			/* AUDIO */
  3,			/* MS */
  0,			/* unused */
  0,			/* string index for interface */

// B.4.2 Class-specific MS Interface Descriptor
  /* descriptor follows inline: */
  7,			/* length of descriptor in bytes */
  36,			/* descriptor type */
  1,			/* header functional descriptor */
  0x0, 0x01,		/* bcdADC */
  65, 0,			/* wTotalLength */

// B.4.3 MIDI IN Jack Descriptor
  /* descriptor follows inline: */
  6,			/* bLength */
  36,			/* descriptor type */
  2,			/* MIDI_IN_JACK desc subtype */
  1,			/* EMBEDDED bJackType */
  1,			/* bJackID */
  0,			/* iJack */

  /* descriptor follows inline: */
  6,			/* bLength */
  36,			/* descriptor type */
  2,			/* MIDI_IN_JACK desc subtype */
  2,			/* EXTERNAL bJackType */
  2,			/* bJackID */
  0,			/* iJack */

//B.4.4 MIDI OUT Jack Descriptor
  /* descriptor follows inline: */
  9,			/* length of descriptor in bytes */
  36,			/* descriptor type */
  3,			/* MIDI_OUT_JACK descriptor */
  1,			/* EMBEDDED bJackType */
  3,			/* bJackID */
  1,			/* No of input pins */
  2,			/* BaSourceID */
  1,			/* BaSourcePin */
  0,			/* iJack */

  /* descriptor follows inline: */
  9,			/* bLength of descriptor in bytes */
  36,			/* bDescriptorType */
  3,			/* MIDI_OUT_JACK bDescriptorSubtype */
  2,			/* EXTERNAL bJackType */
  4,			/* bJackID */
  1,			/* bNrInputPins */
  1,			/* baSourceID (0) */
  1,			/* baSourcePin (0) */
  0,			/* iJack */


// B.5 Bulk OUT Endpoint Descriptors

//B.5.1 Standard Bulk OUT Endpoint Descriptor
  /* descriptor follows inline: */
  9,			/* bLenght */
  USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
  0x1,			/* bEndpointAddress OUT endpoint number 1 */
  3,			/* bmAttributes: 2:Bulk, 3:Interrupt endpoint */
  8, 0,			/* wMaxPacketSize */
  10,			/* bIntervall in ms */
  0,			/* bRefresh */
  0,			/* bSyncAddress */

// B.5.2 Class-specific MS Bulk OUT Endpoint Descriptor
  /* descriptor follows inline: */
  5,			/* bLength of descriptor in bytes */
  37,			/* bDescriptorType */
  1,			/* bDescriptorSubtype */
  1,			/* bNumEmbMIDIJack  */
  1,			/* baAssocJackID (0) */


//B.6 Bulk IN Endpoint Descriptors

//B.6.1 Standard Bulk IN Endpoint Descriptor
  /* descriptor follows inline: */
  9,			/* bLenght */
  USBDESCR_ENDPOINT,	/* bDescriptorType = endpoint */
  0x81,			/* bEndpointAddress IN endpoint number 1 */
  3,			/* bmAttributes: 2: Bulk, 3: Interrupt endpoint */
  8, 0,			/* wMaxPacketSize */
  10,			/* bIntervall in ms */
  0,			/* bRefresh */
  0,			/* bSyncAddress */

// B.6.2 Class-specific MS Bulk IN Endpoint Descriptor
  /* descriptor follows inline: */
  5,			/* bLength of descriptor in bytes */
  37,			/* bDescriptorType */
  1,			/* bDescriptorSubtype */
  1,			/* bNumEmbMIDIJack (0) */
  3,			/* baAssocJackID (0) */
};


uchar usbFunctionDescriptor(usbRequest_t * rq) {
  if (rq->wValue.bytes[1] == USBDESCR_DEVICE) {
    usbMsgPtr = (uchar *) deviceDescrMIDI;
    return sizeof(deviceDescrMIDI);
  } else {		/* must be config descriptor */
    usbMsgPtr = (uchar *) configDescrMIDI;
    return sizeof(configDescrMIDI);
  }
}

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

static uchar    sendEmptyFrame;
// static uchar    modeBuffer[7];

uchar usbFunctionSetup(uchar data[8]) {
  usbRequest_t    *rq = (void *)data;

  if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
    
    /*  Prepare bulk-in endpoint to respond to early termination   */
    if ((rq->bmRequestType & USBRQ_DIR_MASK) ==
	USBRQ_DIR_HOST_TO_DEVICE)
      sendEmptyFrame = 1;
  }
  return 0xff;
}

/*---------------------------------------------------------------------------*/
/* usbFunctionRead                                                           */
/*---------------------------------------------------------------------------*/

uchar usbFunctionRead( uchar *data, uchar len ) {
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
    return 7;
}

/*---------------------------------------------------------------------------*/
/* usbFunctionWrite                                                          */
/*---------------------------------------------------------------------------*/

uchar usbFunctionWrite(uchar * data, uchar len) {
   setPortBit1(&PORTC, 0);		// DEBUG LED
  /* parseMidiMessage(data, len); */
  return 1;
}
/*---------------------------------------------------------------------------*/
/* usbFunctionWriteOut                                                       */
/*                                                                           */
/* this Function is called if a MIDI Out message (from PC) arrives.          */
/*                                                                           */
/*---------------------------------------------------------------------------*/

#ifdef RECEIVER
#define    RX_SIZE        (HW_CDC_BULK_IN_SIZE)
static uchar utxrdy = FALSE;	/* USB Packet ready in utx_buf */
static uchar rx_buf[RX_SIZE];	/* tempory buffer */
static uchar utx_buf[RX_SIZE];	/* BULK_IN buffer */
#endif

#ifdef TRANSMITTER
#define    TX_SIZE        (HW_CDC_BULK_OUT_SIZE<<2)
#define    TX_MASK        (TX_SIZE-1)
static uchar uwptr = 0, irptr = 0;
static uchar tx_buf[TX_SIZE];
#endif

void usbFunctionWriteOut(uchar *data, uchar len ) {
#ifdef TRANSMITTER
  parseUSBMidiMessage(data, len);
#endif
}

#ifdef TRANSMITTER
void parseUSBMidiMessage(uchar *data, uchar len) {
  uchar cin = (*data) & 0x0f;	/* CABLE NOを無視する */
  uchar i;

#ifdef DEBUG
  if (cin == 8) {
    LED_OFF;
  } else if (cin == 9) {
    if (((*(data+3)) & 0x7f) == 0) {
      LED_OFF;
    } else {
      LED_ON;
    }
  }
#else
  FLIP_LED;
#endif

  if (cin > 1) {		/* ignore cin == 0 and cin == 1 */
    for (i = 1 ; i < 4 ; i++) {
      tx_buf[uwptr++] = *(data + i);
      uwptr &= TX_MASK;
      if (i == 1) {
	if ((cin == 5) || /* single byte system common */
	    (cin == 15))  /* single byte */
	  break;
      }
      if (i == 2) {
	if ((cin == 2) ||  /* two-byte system common */
	    (cin == 6) ||  /* system ex end with 2 bytes */
	    (cin == 12) || /* program change */
	    (cin == 13))   /* channel pressure */
	  break;
      }
    }
  }

  if (len > 4) {
    parseUSBMidiMessage(data+4, len-4);
  }
}
#endif

#ifdef RECEIVER

uchar parseSerialMidiMessage(uchar RxByte) {
  static uchar PC = 0;
  static uchar SysEx = FALSE;
  static uchar stateTransTable[] = {
    0, 				/* 0 dummy */
    0,				/* 1 dummy */
    3,				/* 2->3 NOTE OFF (3) */
    2 | 0x80,			/* 3->2 */
    5,				/* 4->5 NOTE ON (3) */
    4 | 0x80,			/* 5->4 */
    7,				/* 6->7 Polyphonic key pressure (3) */
    6 | 0x80,			/* 7->6 */
    9,				/* 8->9 Control Change (3) */
    8 | 0x80,			/* 8->9 */
    10 | 0x80,			/* 10->10 program change (2) */
    0,				/* dummy */
    12 | 0x80,			/* 12->12 Channel Pressure (2) */
    0,				/* 13 dummy */
    15,				/* 14->15 Pitch Bend (3) */
    14 | 0x80			/* 15->14 */
  };

  FLIP_LED;	// DEBUG LED
  if(SysEx){  /* MIDI System Message */
    if(RxByte == 0xf7){		/* MIDI_EndSysEx */
      SysEx = FALSE;
    }
    return FALSE;
  }
  if (RxByte >= 0xF8){		/* Single Byte Message */
    utx_buf[0] = 0x0f;
    utx_buf[1] = RxByte;
    utx_buf[2] = 0;
    utx_buf[3] = 0;
    return TRUE;
  }

  if(RxByte > 0x7F){		/* Channel message */
    if(RxByte == 0xf0){		/* MIDI_StartSysEx */
      SysEx = TRUE;
      return FALSE;
    }
    PC = 0;
  }

  if (PC == 0) {
    PC = (((RxByte >> 4) & 0x07) + 1) * 2;
    // conversion
    // 0x80 -> 2, 0x90 -> 4, 0xa0 -> 6, 0xb0 -> 8, 0xc0 -> 10, 0xd0 -> 12, 0xe0 -> 14
    rx_buf[0] = RxByte >> 4;
    rx_buf[1] = RxByte;
    rx_buf[3] = 0;
  } else {
    uchar tt = stateTransTable[PC];
    rx_buf[(PC & 1) + 2] = RxByte;
    PC = tt & 0x0f;
    if ((tt & 0x80) != 0) {
      memcpy(utx_buf, rx_buf, 4);
      return TRUE;
    }
  }
  return FALSE;
}

#endif

// #ifdef ARDUINO
// #define PORTD_DDR (0x00)
// #else
// //#define PORTD_DDR (0b00000010)
// #endif

static void hardwareInit(void) {
  USB_CFG_IOPORT =(uchar) ~ ((1 << USB_CFG_DMINUS_BIT) |(1 << USB_CFG_DPLUS_BIT));

  /* set baud rate */
  UBRRH = 0;
  UBRRL	= 23;			/* 312500Hz at 16MHz clock */
  /*  */
  UCSRB	= (1<<RXEN) | (1<<TXEN);

  /* DEBUGGING LED */

  DDRC = 0x01;
  
  //serial
  DDRD = 0x02;

}

int main(void) {
  /* WDTを作動させる */
  wdt_enable(WDTO_1S);
  odDebugInit();
  hardwareInit();
  _delay_ms(10);
  usbInit();
  
  sendEmptyFrame = 0;

  sei();
  for(;;){
    wdt_reset();
    usbPoll();
#ifdef TRANSMITTER
    /*    send to Serial MIDI line    */
    if( (UCSRA & (1<<UDRE)) && uwptr!=irptr ) {
      UDR = tx_buf[irptr++];
      irptr &= TX_MASK;
    }
#if USB_CFG_HAVE_FLOWCONTROL == 1
    if( usbAllRequestsAreDisabled() &&
	((uwptr-irptr)&TX_MASK)<(TX_SIZE-HW_CDC_BULK_OUT_SIZE) ) {
      usbEnableAllRequests();
    }
#endif
#endif

#ifdef RECEIVER
    /*    receive from Serial MIDI line    */
    if( UCSRA & (1<<RXC)) {
      utxrdy |= parseSerialMidiMessage(UDR);
    }
    /* send packets to USB MIDI  */
    if( usbInterruptIsReady() && utxrdy ) {
      usbSetInterrupt(utx_buf, 4);
      utxrdy = FALSE;
    }
#endif
  }
}

/* EOF */
