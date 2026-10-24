//! The bytes NXP's McuBoot client puts in front of the ROM, against UM11126's example frames.
//!
//! The expected frames are copied from UM11126 Rev 2.8, chapter 8, and each one's CRC was
//! checked against its own bytes first. Figure 29 prints its byte count as `00 01 00 00`; its CRC
//! and its caption both say `0x1000`, so the frame here uses `00 10 00 00`.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::time::Duration;

use mboot::McuBoot;
use mboot::protocols::{CommunicationError, Protocol};
use mboot::tags::property::PropertyTagDiscriminants;

type Sent = Rc<RefCell<Vec<Vec<u8>>>>;

/// Records what it is sent and answers from a queue.
struct Recorder {
    sent: Sent,
    replies: VecDeque<Vec<u8>>,
}

/// A client whose replies are `replies`, and the log of what it sends.
fn client(replies: &[&str]) -> (McuBoot<Recorder>, Sent) {
    let sent = Sent::default();
    let recorder = Recorder {
        sent: sent.clone(),
        replies: replies.iter().map(|r| hex(r)).collect(),
    };
    (McuBoot::new(recorder), sent)
}

fn hex(text: &str) -> Vec<u8> {
    text.split_whitespace()
        .map(|b| u8::from_str_radix(b, 16).unwrap())
        .collect()
}

impl Protocol for Recorder {
    fn get_timeout(&self) -> Duration {
        Duration::from_secs(1)
    }
    fn get_polling_interval(&self) -> Duration {
        Duration::from_millis(1)
    }
    fn get_identifier(&self) -> &str {
        "recorder"
    }
    fn read(&mut self, _: usize) -> Result<Vec<u8>, CommunicationError> {
        Err(CommunicationError::UnsupportedPlatform)
    }
    fn write_packet_raw(&mut self, data: &[u8]) -> Result<(), CommunicationError> {
        self.sent.borrow_mut().push(data.to_vec());
        Ok(())
    }
    fn read_packet_raw(&mut self, _: u8) -> Result<Vec<u8>, CommunicationError> {
        // The replies are framed as UM11126 prints them; the client wants the payload.
        let frame = self
            .replies
            .pop_front()
            .ok_or(CommunicationError::Timeout)?;
        Ok(frame[6..].to_vec())
    }
}

#[test]
fn get_property_is_figure_26() {
    let (mut boot, sent) = client(&["5a a4 0c 00 64 18 a7 00 00 02 00 00 00 00 00 01 02 4b"]);
    let reply = boot
        .get_property(PropertyTagDiscriminants::CurrentVersion, 0)
        .unwrap();
    assert_eq!(reply.response_words[0], 0x4B02_0100);
    assert_eq!(
        *sent.borrow(),
        [hex("5a a4 0c 00 4b 33 07 00 00 02 01 00 00 00 00 00 00 00")]
    );
}

#[test]
fn flash_erase_region_is_figure_29() {
    let (mut boot, sent) = client(&["5a a4 0c 00 ba 55 a0 00 00 02 00 00 00 00 02 00 00 00"]);
    boot.flash_erase_region(0, 0x1000, 0).unwrap();
    assert_eq!(
        *sent.borrow(),
        [hex(
            "5a a4 10 00 c5 f0 02 00 00 03 00 00 00 00 00 10 00 00 00 00 00 00"
        )]
    );
}

#[test]
fn read_memory_is_figure_30() {
    let (mut boot, sent) = client(&[
        "5a a4 0c 00 a3 7e a3 01 00 02 00 00 00 00 10 00 00 00",
        "5a a5 10 00 99 5b d1 00 20 40 11 2e 01 20 00 00 00 00 00 00 00 00",
        "5a a4 0c 00 0e 23 a0 00 00 02 00 00 00 00 03 00 00 00",
    ]);
    let read = boot.read_memory(0x2000_0400, 0x10, 0).unwrap();
    assert_eq!(
        &*read.bytes,
        hex("d1 00 20 40 11 2e 01 20 00 00 00 00 00 00 00 00").as_slice()
    );
    assert_eq!(
        *sent.borrow(),
        [hex(
            "5a a4 10 00 95 a4 03 00 00 03 00 04 00 20 10 00 00 00 00 00 00 00"
        )]
    );
}

#[test]
fn write_memory_is_figure_31() {
    let data = hex("00 11 22 33 44 55 66 77 88 99 aa bb cc dd ee ff");
    let (mut boot, sent) = client(&[
        // The client asks for the packet size before every write. Not a UM11126 frame: 56 is the
        // USB payload UM11126 gives, and the CRC bytes are zero because nothing here reads them.
        "5a a4 0c 00 00 00 a7 00 00 02 00 00 00 00 38 00 00 00",
        "5a a4 0c 00 23 72 a0 00 00 02 00 00 00 00 04 00 00 00",
        "5a a4 0c 00 23 72 a0 00 00 02 00 00 00 00 04 00 00 00",
    ]);
    boot.write_memory(0x2000_0400, 0, &data).unwrap();
    let sent = sent.borrow();
    assert_eq!(sent.len(), 3);
    // sent[0] is the packet size query.
    assert_eq!(sent[0][6..10], [0x07, 0x00, 0x00, 0x02]);
    assert_eq!(
        sent[1],
        hex("5a a4 10 00 f6 62 04 01 00 03 00 04 00 20 10 00 00 00 00 00 00 00")
    );
    assert_eq!(
        sent[2],
        hex("5a a5 10 00 25 84 00 11 22 33 44 55 66 77 88 99 aa bb cc dd ee ff")
    );
}

#[test]
fn reset_is_figure_34() {
    let (mut boot, sent) = client(&["5a a4 0c 00 cd a6 a0 00 00 02 00 00 00 00 0b 00 00 00"]);
    boot.reset().unwrap();
    assert_eq!(*sent.borrow(), [hex("5a a4 04 00 6f 46 0b 00 00 00")]);
}
