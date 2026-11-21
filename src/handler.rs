//! Interrupt dispatch: vector -> registered handler -> EOI.
//!
//! The CPU-facing stub for a vector (installed in the IDT by the
//! kernel) calls [`dispatch`]. Dispatch looks the vector up in the
//! [`HandlerTable`], calls the handler, then acknowledges the interrupt
//! through an [`Eoi`] sink so the controller can deliver the next one.
//!
//! EOI routing:
//! * vectors 32–39 — PIC master only;
//! * vectors 40–47 — PIC slave, then master;
//! * vectors 48+ — local APIC EOI;
//! * anything else (CPU exceptions) — no EOI; exceptions are not
//!   level-triggered IRQ lines.
//!
//! Spurious vectors (39 = IRQ7, 47 = IRQ15) with *no* handler are
//! reported as [`DispatchOutcome::Spurious`] and get no EOI from
//! `dispatch` — a spurious IRQ7 needs no EOI at all, and a spurious
//! IRQ15 needs an EOI to the master only, which the caller sends via
//! `Pic::send_eoi_checked` after reading the ISR. Sending a blind EOI
//! for a spurious IRQ7 can lose a real IRQ on some hardware.

use crate::apic::LocalApic;
use crate::apic::RegWindow;
use crate::idt::{HandlerTable, APIC_VECTOR_BASE, PIC_IRQ_BASE, SPURIOUS_MASTER_VECTOR, SPURIOUS_SLAVE_VECTOR};
use crate::pic::Pic;
use crate::ports::PortIo;

/// Something that can acknowledge an interrupt for `vector`.
pub trait Eoi {
    fn eoi(&mut self, vector: u8);
}

impl<P: PortIo> Eoi for Pic<P> {
    fn eoi(&mut self, vector: u8) {
        if (PIC_IRQ_BASE..PIC_IRQ_BASE + 16).contains(&vector) {
            self.send_eoi(vector - PIC_IRQ_BASE);
        }
    }
}

impl<R: RegWindow> Eoi for LocalApic<R> {
    fn eoi(&mut self, vector: u8) {
        if vector >= APIC_VECTOR_BASE {
            LocalApic::eoi(self);
        }
    }
}

/// Routes EOI to the PIC (vectors 32–47) or the local APIC (48+).
pub struct ChainedEoi<P: PortIo, R: RegWindow> {
    pic: Pic<P>,
    apic: LocalApic<R>,
}

impl<P: PortIo, R: RegWindow> ChainedEoi<P, R> {
    pub fn new(pic: Pic<P>, apic: LocalApic<R>) -> Self { Self { pic, apic } }
    pub fn pic(&mut self) -> &mut Pic<P> { &mut self.pic }
    pub fn apic(&mut self) -> &mut LocalApic<R> { &mut self.apic }
}

impl<P: PortIo, R: RegWindow> Eoi for ChainedEoi<P, R> {
    fn eoi(&mut self, vector: u8) {
        if vector >= APIC_VECTOR_BASE {
            self.apic.eoi();
        } else if vector >= PIC_IRQ_BASE {
            self.pic.send_eoi(vector - PIC_IRQ_BASE);
        }
    }
}

/// Records every EOI vector it receives (up to 64). Test helper and a
/// cheap way for a kernel to prove dispatch acknowledged an interrupt.
pub struct RecordingEoi {
    vectors: [u8; 64],
    len: usize,
}

impl RecordingEoi {
    pub fn new() -> Self { Self { vectors: [0; 64], len: 0 } }
    pub fn vectors(&self) -> &[u8] { &self.vectors[..self.len] }
    pub fn len(&self) -> usize { self.len }
    pub fn is_empty(&self) -> bool { self.len == 0 }
}

impl Default for RecordingEoi {
    fn default() -> Self { Self::new() }
}

impl Eoi for RecordingEoi {
    fn eoi(&mut self, vector: u8) {
        if self.len < self.vectors.len() {
            self.vectors[self.len] = vector;
            self.len += 1;
        }
    }
}

/// What `dispatch` did with an interrupt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DispatchOutcome {
    /// A handler was registered and ran; EOI was sent.
    Handled,
    /// No handler registered. EOI was still sent (except for the
    /// spurious vectors), so the controller does not wedge.
    NoHandler,
    /// Spurious IRQ7/IRQ15 with no handler: no EOI sent here; see the
    /// module docs for who owes what.
    Spurious,
}

/// Look `vector` up in `table`, run its handler if any, then send EOI
/// through `eoi`. The handler receives the vector number.
pub fn dispatch(vector: u8, table: &HandlerTable, eoi: &mut dyn Eoi) -> DispatchOutcome {
    match table.get(vector) {
        Some(handler) => {
            handler(vector);
            eoi.eoi(vector);
            DispatchOutcome::Handled
        }
        None => {
            if vector == SPURIOUS_MASTER_VECTOR || vector == SPURIOUS_SLAVE_VECTOR {
                DispatchOutcome::Spurious
            } else {
                eoi.eoi(vector);
                DispatchOutcome::NoHandler
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::apic::MockRegWindow;
    use crate::idt::TIMER_VECTOR;
    use crate::ports::MockPorts;
    use core::sync::atomic::{AtomicU8, Ordering};

    #[test]
    fn dispatch_calls_the_registered_handler_and_eois() {
        static SEEN: AtomicU8 = AtomicU8::new(0);
        fn handler(v: u8) {
            SEEN.store(v, Ordering::SeqCst);
        }
        let mut table = HandlerTable::new();
        table.register(TIMER_VECTOR, handler);
        let mut eoi = RecordingEoi::new();
        let out = dispatch(TIMER_VECTOR, &table, &mut eoi);
        assert_eq!(out, DispatchOutcome::Handled);
        assert_eq!(SEEN.load(Ordering::SeqCst), TIMER_VECTOR);
        assert_eq!(eoi.vectors(), &[TIMER_VECTOR]);
    }

    #[test]
    fn dispatch_without_handler_still_eois() {
        let table = HandlerTable::new();
        let mut eoi = RecordingEoi::new();
        let out = dispatch(33, &table, &mut eoi);
        assert_eq!(out, DispatchOutcome::NoHandler);
        assert_eq!(eoi.vectors(), &[33]);
    }

    #[test]
    fn dispatch_spurious_vector_sends_no_eoi() {
        let table = HandlerTable::new();
        let mut eoi = RecordingEoi::new();
        assert_eq!(dispatch(39, &table, &mut eoi), DispatchOutcome::Spurious);
        assert_eq!(dispatch(47, &table, &mut eoi), DispatchOutcome::Spurious);
        assert!(eoi.is_empty());
    }

    #[test]
    fn dispatch_spurious_vector_with_handler_is_handled() {
        static SEEN: AtomicU8 = AtomicU8::new(0);
        fn handler(v: u8) {
            SEEN.store(v, Ordering::SeqCst);
        }
        let mut table = HandlerTable::new();
        table.register(39, handler);
        let mut eoi = RecordingEoi::new();
        assert_eq!(dispatch(39, &table, &mut eoi), DispatchOutcome::Handled);
        assert_eq!(SEEN.load(Ordering::SeqCst), 39);
        assert_eq!(eoi.vectors(), &[39]);
    }

    #[test]
    fn pic_eoi_routing_by_vector() {
        let mut pic = Pic::new(MockPorts::new());
        // Timer (vector 32, IRQ0): master EOI only.
        Pic::eoi(&mut pic, 32);
        assert_eq!(pic.ports().write_count(crate::pic::PIC1_COMMAND, crate::pic::EOI), 1);
        assert_eq!(pic.ports().write_count(crate::pic::PIC2_COMMAND, crate::pic::EOI), 0);
        // RTC (vector 40, IRQ8): slave then master.
        let mut pic = Pic::new(MockPorts::new());
        Pic::eoi(&mut pic, 40);
        assert_eq!(pic.ports().write_count(crate::pic::PIC2_COMMAND, crate::pic::EOI), 1);
        assert_eq!(pic.ports().write_count(crate::pic::PIC1_COMMAND, crate::pic::EOI), 1);
        // APIC vector: the PIC must stay silent.
        let mut pic = Pic::new(MockPorts::new());
        Pic::eoi(&mut pic, 48);
        assert_eq!(pic.ports().write_log_len(), 0);
        // Exception vector: no EOI at all.
        let mut pic = Pic::new(MockPorts::new());
        Pic::eoi(&mut pic, 14);
        assert_eq!(pic.ports().write_log_len(), 0);
    }

    #[test]
    fn chained_eoi_routes_pic_and_apic() {
        let pic = Pic::new(MockPorts::new());
        let apic = LocalApic::new(MockRegWindow::new());
        let mut chain = ChainedEoi::new(pic, apic);
        chain.eoi(32); // PIC timer
        assert_eq!(chain.pic().ports().write_count(crate::pic::PIC1_COMMAND, crate::pic::EOI), 1);
        assert_eq!(chain.apic().regs().get(crate::apic::lapic::EOI), 0); // untouched
        chain.eoi(48); // APIC vector
        // APIC EOI writes 0 to the EOI register; PIC untouched this time.
        assert_eq!(chain.apic().regs().get(crate::apic::lapic::EOI), 0);
        assert_eq!(chain.pic().ports().write_count(crate::pic::PIC1_COMMAND, crate::pic::EOI), 1);
    }

    #[test]
    fn dispatch_end_to_end_with_pic() {
        static SEEN: AtomicU8 = AtomicU8::new(0);
        fn handler(v: u8) {
            SEEN.store(v, Ordering::SeqCst);
        }
        let mut table = HandlerTable::new();
        table.register(32, handler);
        let mut pic = Pic::new(MockPorts::new());
        let out = dispatch(32, &table, &mut pic);
        assert_eq!(out, DispatchOutcome::Handled);
        assert_eq!(SEEN.load(Ordering::SeqCst), 32);
        assert!(pic.ports().wrote(crate::pic::PIC1_COMMAND, crate::pic::EOI));
    }
}
