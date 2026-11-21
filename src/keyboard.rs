//! Keyboard IRQ handler: PS/2 scancode set 1 -> decoded key events.
//!
//! IRQ1 (vector 33) fires when the PS/2 controller has a byte ready.
//! [`KeyboardIrq::on_irq`] reads it from port `0x60`, decodes it with
//! [`KeyboardDecoder`], and pushes the resulting [`KeyEvent`] into a
//! fixed 64-entry ring buffer ([`KeyRing`]) the kernel/shell drains.
//! EOI is the caller's job and is never skipped: a full ring drops the
//! *event* (counted in [`KeyRing::dropped`]) but the interrupt is still
//! acknowledged, so the controller never wedges.
//!
//! The scancode table mirrors `rustos-drivers::keyboard` (same set-1
//! make codes, same shift/caps rules) — the two tables must agree, and
//! the tests here assert the same values the drivers crate asserts
//! (`0x1E` -> `a`, shift+`0x1E` -> `A`, `0x02` -> `1`/`!`). This crate
//! keeps its own copy instead of depending on `rustos-drivers` because
//! the decoder here produces full key *events* (make/break, extended
//! keys, modifiers), where the drivers crate only yields printable
//! characters. If the table ever changes, change it in both places.
//!
//! Decoder state machine (set 1, as the 8042 delivers it):
//!
//! * `0xE0` — extended prefix: the next code is an extended key
//!   (arrows, Insert/Delete, …). It yields an event with
//!   `extended = true` and no character.
//! * `0xF0` — break prefix (the `0xE0 0xF0 xx` form): the next code is
//!   a key release.
//! * Otherwise a code with bit 7 set is a release (set-1 break form),
//!   a code with bit 7 clear is a make. Shift make/release updates
//!   `shift`; Caps Lock toggles `caps_lock` on make only.

use crate::ports::PortIo;

/// PS/2 data port: read the scancode here on IRQ1.
pub const PS2_DATA_PORT: u16 = 0x60;
/// PS/2 status/command port: bit 0 = output buffer full (a byte waits).
pub const PS2_STATUS_PORT: u16 = 0x64;
/// Status-port bit: output buffer full.
pub const PS2_STATUS_OUTPUT_FULL: u8 = 0x01;

/// Ring-buffer capacity for decoded key events.
pub const KEY_RING_CAPACITY: usize = 64;

const SCANCODE_SHIFT_LEFT: u8 = 0x2A;
const SCANCODE_SHIFT_RIGHT: u8 = 0x36;
const SCANCODE_CAPS_LOCK: u8 = 0x3A;
const SCANCODE_EXTENDED: u8 = 0xE0;
const SCANCODE_BREAK_PREFIX: u8 = 0xF0;

/// Map a make-code to (unshifted, shifted) ASCII. `None` = not a
/// printable key (function keys, arrows, …).
fn scancode_map(code: u8) -> Option<(char, char)> {
    Some(match code {
        0x02 => ('1', '!'), 0x03 => ('2', '@'), 0x04 => ('3', '#'),
        0x05 => ('4', '$'), 0x06 => ('5', '%'), 0x07 => ('6', '^'),
        0x08 => ('7', '&'), 0x09 => ('8', '*'), 0x0A => ('9', '('),
        0x0B => ('0', ')'), 0x0C => ('-', '_'), 0x0D => ('=', '+'),
        0x0E => ('\u{8}', '\u{8}'), 0x0F => ('\t', '\t'),
        0x10 => ('q', 'Q'), 0x11 => ('w', 'W'), 0x12 => ('e', 'E'),
        0x13 => ('r', 'R'), 0x14 => ('t', 'T'), 0x15 => ('y', 'Y'),
        0x16 => ('u', 'U'), 0x17 => ('i', 'I'), 0x18 => ('o', 'O'),
        0x19 => ('p', 'P'), 0x1A => ('[', '{'), 0x1B => (']', '}'),
        0x1C => ('\n', '\n'),
        0x1E => ('a', 'A'), 0x1F => ('s', 'S'), 0x20 => ('d', 'D'),
        0x21 => ('f', 'F'), 0x22 => ('g', 'G'), 0x23 => ('h', 'H'),
        0x24 => ('j', 'J'), 0x25 => ('k', 'K'), 0x26 => ('l', 'L'),
        0x27 => (';', ':'), 0x28 => ('\'', '"'), 0x29 => ('`', '~'),
        0x2B => ('\\', '|'),
        0x2C => ('z', 'Z'), 0x2D => ('x', 'X'), 0x2E => ('c', 'C'),
        0x2F => ('v', 'V'), 0x30 => ('b', 'B'), 0x31 => ('n', 'N'),
        0x32 => ('m', 'M'), 0x33 => (',', '<'), 0x34 => ('.', '>'),
        0x35 => ('/', '?'), 0x39 => (' ', ' '),
        _ => return None,
    })
}

/// One decoded key press or release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyEvent {
    /// Make code (bit 7 always clear), e.g. `0x1E` for `a`.
    pub scancode: u8,
    /// `true` = key pressed (make), `false` = key released (break).
    pub pressed: bool,
    /// `true` when the code arrived after a `0xE0` prefix.
    pub extended: bool,
    /// Printable character, on make only (`None` for releases,
    /// modifiers, and non-printable keys).
    pub ch: Option<char>,
    /// Shift held when this event was produced.
    pub shift: bool,
    /// Caps Lock active when this event was produced.
    pub caps_lock: bool,
}

/// PS/2 set-1 decoder: raw scancode bytes in, [`KeyEvent`]s out.
/// Holds shift / caps-lock / extended / break-prefix state.
#[derive(Debug, Clone, Copy, Default)]
pub struct KeyboardDecoder {
    shift: bool,
    caps_lock: bool,
    extended: bool,
    release_next: bool,
}

impl KeyboardDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Is either Shift key currently held?
    pub fn shift(&self) -> bool {
        self.shift
    }

    /// Is Caps Lock currently on?
    pub fn caps_lock(&self) -> bool {
        self.caps_lock
    }

    /// Feed one scancode byte. Returns the event it completes, if any.
    /// Prefix bytes (`0xE0`, `0xF0`) update state and return `None`.
    pub fn handle_scancode(&mut self, code: u8) -> Option<KeyEvent> {
        if code == SCANCODE_EXTENDED {
            self.extended = true;
            return None;
        }
        if code == SCANCODE_BREAK_PREFIX {
            self.release_next = true;
            return None;
        }
        let extended = self.extended;
        self.extended = false;
        let release_prefix = self.release_next;
        self.release_next = false;

        // A release is either the 0xF0 prefix or bit 7 set (set-1 form).
        let pressed = !release_prefix && code & 0x80 == 0;
        let scancode = code & 0x7F;

        // Modifier keys update state; they still produce an event so
        // the consumer can track modifiers, but never a character.
        match scancode {
            SCANCODE_SHIFT_LEFT | SCANCODE_SHIFT_RIGHT => {
                self.shift = pressed;
                return Some(KeyEvent {
                    scancode,
                    pressed,
                    extended,
                    ch: None,
                    shift: self.shift,
                    caps_lock: self.caps_lock,
                });
            }
            SCANCODE_CAPS_LOCK if pressed && !extended => {
                self.caps_lock = !self.caps_lock;
                return Some(KeyEvent {
                    scancode,
                    pressed,
                    extended,
                    ch: None,
                    shift: self.shift,
                    caps_lock: self.caps_lock,
                });
            }
            _ => {}
        }

        let ch = if pressed && !extended {
            scancode_map(scancode).map(|(plain, shifted)| {
                let upper = self.shift ^ self.caps_lock;
                if plain.is_ascii_alphabetic() {
                    if upper { shifted } else { plain }
                } else if self.shift {
                    shifted
                } else {
                    plain
                }
            })
        } else {
            None
        };

        Some(KeyEvent {
            scancode,
            pressed,
            extended,
            ch,
            shift: self.shift,
            caps_lock: self.caps_lock,
        })
    }
}

/// Fixed-size ring buffer for [`KeyEvent`]s. No allocation: the kernel
/// drains it from the idle loop. When full, `push` drops the new event
/// and counts it in `dropped` — the buffer never blocks the IRQ path.
#[derive(Debug, Clone, Copy)]
pub struct KeyRing {
    buf: [Option<KeyEvent>; KEY_RING_CAPACITY],
    head: usize,
    tail: usize,
    len: usize,
    dropped: u64,
}

impl KeyRing {
    pub fn new() -> Self {
        Self {
            buf: [None; KEY_RING_CAPACITY],
            head: 0,
            tail: 0,
            len: 0,
            dropped: 0,
        }
    }

    /// Push an event. Returns `false` (and counts a drop) when full.
    pub fn push(&mut self, event: KeyEvent) -> bool {
        if self.len == KEY_RING_CAPACITY {
            self.dropped += 1;
            return false;
        }
        self.buf[self.tail] = Some(event);
        self.tail = (self.tail + 1) % KEY_RING_CAPACITY;
        self.len += 1;
        true
    }

    /// Pop the oldest event, or `None` when empty.
    pub fn pop(&mut self) -> Option<KeyEvent> {
        if self.len == 0 {
            return None;
        }
        let event = self.buf[self.head].take();
        self.head = (self.head + 1) % KEY_RING_CAPACITY;
        self.len -= 1;
        event
    }

    /// Events currently buffered.
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Events dropped because the ring was full.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }
}

impl Default for KeyRing {
    fn default() -> Self {
        Self::new()
    }
}

/// The IRQ1 handler: ports + decoder + ring, in one place.
///
/// The kernel's vector-33 stub calls [`KeyboardIrq::on_irq`], then
/// sends EOI. `on_irq` never panics and never blocks: if no byte is
/// waiting it returns `false` immediately, and a full ring drops the
/// event instead of stalling.
pub struct KeyboardIrq<P: PortIo> {
    ports: P,
    decoder: KeyboardDecoder,
    ring: KeyRing,
}

impl<P: PortIo> KeyboardIrq<P> {
    pub fn new(ports: P) -> Self {
        Self { ports, decoder: KeyboardDecoder::new(), ring: KeyRing::new() }
    }

    /// Mutable access to the underlying port I/O (tests, mainly).
    pub fn ports(&mut self) -> &mut P {
        &mut self.ports
    }

    /// Handle one IRQ1: if the PS/2 output buffer holds a byte, read
    /// port `0x60`, decode it, and queue the event. Returns `true`
    /// when a byte was consumed. EOI is the caller's job.
    pub fn on_irq(&mut self) -> bool {
        if self.ports.read_u8(PS2_STATUS_PORT) & PS2_STATUS_OUTPUT_FULL == 0 {
            return false;
        }
        let code = self.ports.read_u8(PS2_DATA_PORT);
        if let Some(event) = self.decoder.handle_scancode(code) {
            self.ring.push(event);
        }
        true
    }

    /// Poll once: same as [`KeyboardIrq::on_irq`]; for callers that
    /// drive the keyboard without an IRQ (early boot, tests).
    pub fn poll(&mut self) -> bool {
        self.on_irq()
    }

    /// Pop the oldest queued event, or `None` when empty.
    pub fn pop_event(&mut self) -> Option<KeyEvent> {
        self.ring.pop()
    }

    /// Events currently queued.
    pub fn pending(&self) -> usize {
        self.ring.len()
    }

    /// Events dropped because the ring was full.
    pub fn dropped(&self) -> u64 {
        self.ring.dropped()
    }

    /// Is Shift currently held?
    pub fn shift(&self) -> bool {
        self.decoder.shift()
    }

    /// Is Caps Lock currently on?
    pub fn caps_lock(&self) -> bool {
        self.decoder.caps_lock()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ports::MockPorts;

    fn feed(decoder: &mut KeyboardDecoder, codes: &[u8]) -> Vec<KeyEvent> {
        codes.iter().filter_map(|&c| decoder.handle_scancode(c)).collect()
    }

    #[test]
    fn make_break_and_shift() {
        let mut d = KeyboardDecoder::new();
        let evs = feed(&mut d, &[0x1E]);
        assert_eq!(evs.len(), 1);
        assert_eq!(evs[0].ch, Some('a'));
        assert!(evs[0].pressed);
        assert!(!evs[0].extended);
        // Release: event, no character.
        let evs = feed(&mut d, &[0x9E]);
        assert_eq!(evs.len(), 1);
        assert!(!evs[0].pressed);
        assert_eq!(evs[0].ch, None);
        assert_eq!(evs[0].scancode, 0x1E);
        // Shift + a = A; shift+1 = !.
        let evs = feed(&mut d, &[0x2A, 0x1E, 0x02, 0xAA, 0x1E]);
        assert_eq!(evs[0].ch, None); // shift make itself
        assert!(evs[0].pressed);
        assert_eq!(evs[1].ch, Some('A'));
        assert_eq!(evs[2].ch, Some('!'));
        assert!(!evs[3].pressed); // shift release
        assert_eq!(evs[4].ch, Some('a'));
        assert!(d.shift() == false);
    }

    #[test]
    fn f0_break_prefix_form() {
        let mut d = KeyboardDecoder::new();
        let evs = feed(&mut d, &[0x1E, 0xF0, 0x1E, 0x1E]);
        assert_eq!(evs.len(), 3);
        assert!(evs[0].pressed);
        assert!(!evs[1].pressed); // 0xF0 0x1E release
        assert!(evs[2].pressed);
        // Shift via 0xF0 form.
        let evs = feed(&mut d, &[0x2A, 0xF0, 0x2A]);
        assert!(d.shift() == false);
        assert_eq!(evs.len(), 2);
        assert!(!evs[1].pressed);
    }

    #[test]
    fn caps_lock_toggles_and_combines_with_shift() {
        let mut d = KeyboardDecoder::new();
        let evs = feed(&mut d, &[0x3A, 0x1E, 0x3A, 0x1E]);
        assert_eq!(evs[1].ch, Some('A'));
        assert_eq!(evs[3].ch, Some('a'));
        // Shift + caps = lowercase.
        let evs = feed(&mut d, &[0x3A, 0x2A, 0x1E, 0xAA]);
        assert_eq!(evs[2].ch, Some('a'));
    }

    #[test]
    fn extended_codes_produce_extended_events_without_chars() {
        let mut d = KeyboardDecoder::new();
        // Up arrow: 0xE0 0x48 make, 0xE0 0xF0 0x48 break.
        let evs = feed(&mut d, &[0xE0, 0x48]);
        assert_eq!(evs.len(), 1);
        assert!(evs[0].extended);
        assert!(evs[0].pressed);
        assert_eq!(evs[0].ch, None);
        assert_eq!(evs[0].scancode, 0x48);
        let evs = feed(&mut d, &[0xE0, 0xF0, 0x48]);
        assert_eq!(evs.len(), 1);
        assert!(evs[0].extended);
        assert!(!evs[0].pressed);
        // State recovers: plain keys still decode after extended ones.
        let evs = feed(&mut d, &[0x1E]);
        assert_eq!(evs[0].ch, Some('a'));
        assert!(!evs[0].extended);
    }

    #[test]
    fn digits_punctuation_space_enter_and_backspace() {
        let mut d = KeyboardDecoder::new();
        let evs = feed(&mut d, &[0x02, 0x39, 0x1C, 0x0E, 0x27]);
        assert_eq!(
            evs.iter().map(|e| e.ch).collect::<Vec<_>>(),
            vec![Some('1'), Some(' '), Some('\n'), Some('\u{8}'), Some(';')]
        );
        let evs = feed(&mut d, &[0x2A, 0x27, 0xAA]);
        assert_eq!(evs[1].ch, Some(':'));
    }

    #[test]
    fn unknown_codes_are_events_without_chars() {
        let mut d = KeyboardDecoder::new();
        let evs = feed(&mut d, &[0xFF, 0x1E]);
        assert_eq!(evs.len(), 2);
        assert_eq!(evs[0].ch, None);
        assert_eq!(evs[1].ch, Some('a'));
    }

    #[test]
    fn ring_fifo_order_and_overflow_drops_newest() {
        let mut ring = KeyRing::new();
        let ev = KeyEvent {
            scancode: 0x1E,
            pressed: true,
            extended: false,
            ch: Some('a'),
            shift: false,
            caps_lock: false,
        };
        for _ in 0..KEY_RING_CAPACITY {
            assert!(ring.push(ev));
        }
        assert_eq!(ring.len(), KEY_RING_CAPACITY);
        assert!(!ring.push(ev)); // full: drop, count it
        assert_eq!(ring.dropped(), 1);
        assert!(!ring.push(ev));
        assert_eq!(ring.dropped(), 2);
        // FIFO: all 64 pop in order, then empty.
        for _ in 0..KEY_RING_CAPACITY {
            assert_eq!(ring.pop(), Some(ev));
        }
        assert_eq!(ring.pop(), None);
        assert!(ring.is_empty());
    }

    #[test]
    fn irq_reads_port_and_queues_chars() {
        let mut kb = KeyboardIrq::new(MockPorts::new());
        assert!(!kb.on_irq()); // status bit clear: nothing consumed
        assert_eq!(kb.pending(), 0);
        // Feed 'a': status full + data 0x1E.
        kb.ports().set(PS2_STATUS_PORT, 0x01);
        kb.ports().set(PS2_DATA_PORT, 0x1E);
        assert!(kb.on_irq());
        assert_eq!(kb.pending(), 1);
        assert_eq!(kb.pop_event().unwrap().ch, Some('a'));
        // Feed shift + 'b' across two IRQs.
        kb.ports().set(PS2_STATUS_PORT, 0x01);
        kb.ports().set(PS2_DATA_PORT, 0x2A);
        assert!(kb.on_irq());
        kb.ports().set(PS2_STATUS_PORT, 0x01);
        kb.ports().set(PS2_DATA_PORT, 0x30);
        assert!(kb.on_irq());
        assert!(kb.shift());
        assert_eq!(kb.pop_event().unwrap().ch, None); // shift make
        assert_eq!(kb.pop_event().unwrap().ch, Some('B'));
    }

    #[test]
    fn irq_overflow_drops_events_but_still_consumes_bytes() {
        let mut kb = KeyboardIrq::new(MockPorts::new());
        // Fill the ring with 'a' presses (make + break alternate so
        // the decoder keeps producing events).
        for i in 0..KEY_RING_CAPACITY {
            let code = if i % 2 == 0 { 0x1E } else { 0x9E };
            kb.ports().set(PS2_STATUS_PORT, 0x01);
            kb.ports().set(PS2_DATA_PORT, code);
            assert!(kb.on_irq());
        }
        assert_eq!(kb.pending(), KEY_RING_CAPACITY);
        // One more IRQ: byte is still consumed (true) even though the
        // event is dropped — the caller still sends EOI either way.
        kb.ports().set(PS2_STATUS_PORT, 0x01);
        kb.ports().set(PS2_DATA_PORT, 0x1E);
        assert!(kb.on_irq());
        assert_eq!(kb.dropped(), 1);
        assert_eq!(kb.pending(), KEY_RING_CAPACITY);
    }
}
