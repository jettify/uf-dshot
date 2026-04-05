use core::future::poll_fn;
use core::ptr;
use core::sync::atomic::{AtomicPtr, AtomicU8, Ordering};

use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{with_timeout, Duration};

use super::driver::DshotError;

const DMA_IRQ_SLOTS: usize = 16;

pub(crate) type DmaIrqFn = unsafe fn(*mut ());

struct IrqSlot {
    func: AtomicPtr<()>,
    ctx: AtomicPtr<()>,
}

impl IrqSlot {
    const fn new() -> Self {
        Self {
            func: AtomicPtr::new(ptr::null_mut()),
            ctx: AtomicPtr::new(ptr::null_mut()),
        }
    }

    fn install(&self, ctx: *mut (), irq_fn: DmaIrqFn) {
        // Store ctx first so it is visible before func (the presence flag).
        self.ctx.store(ctx, Ordering::Release);
        self.func.store(irq_fn as *mut (), Ordering::Release);
    }

    fn clear(&self) {
        self.func.store(ptr::null_mut(), Ordering::Release);
        self.ctx.store(ptr::null_mut(), Ordering::Release);
    }

    /// Dispatches the installed IRQ callback, if present.
    ///
    /// # Safety
    /// The registered `ctx` pointer must still be valid and associated with `irq_fn`.
    #[inline(always)]
    unsafe fn dispatch(&self) {
        let func = self.func.load(Ordering::Acquire);
        if !func.is_null() {
            let ctx = self.ctx.load(Ordering::Acquire);
            let f: DmaIrqFn = unsafe { core::mem::transmute(func) };
            f(ctx);
        }
    }
}

static DMA_IRQ_SLOTS_TABLE: [IrqSlot; DMA_IRQ_SLOTS] = [const { IrqSlot::new() }; DMA_IRQ_SLOTS];

pub(crate) fn install_irq_slot(slot: usize, ctx: *mut (), irq_fn: DmaIrqFn) {
    DMA_IRQ_SLOTS_TABLE[slot].install(ctx, irq_fn);
}

pub(crate) fn clear_irq_slot(slot: usize) {
    DMA_IRQ_SLOTS_TABLE[slot].clear();
}

/// Dispatches the handler stored in the given slot.
///
/// # Safety
/// The slot's registered context pointer must remain valid for the callback duration.
pub(crate) unsafe fn dispatch_irq_slot(slot: usize) {
    DMA_IRQ_SLOTS_TABLE[slot].dispatch();
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum IrqPhase {
    Idle = 0,
    TxActive = 1,
    RxActive = 2,
    Done = 3,
    TxError = 4,
    RxError = 5,
}

impl IrqPhase {
    fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::TxActive,
            2 => Self::RxActive,
            3 => Self::Done,
            4 => Self::TxError,
            5 => Self::RxError,
            _ => Self::TxError,
        }
    }
}

enum PhaseDisposition {
    Pending,
    Done,
    Error(DshotError),
}

pub(crate) struct IrqWakerState {
    phase: AtomicU8,
    waker: AtomicWaker,
}

impl IrqWakerState {
    pub(crate) const fn new() -> Self {
        Self {
            phase: AtomicU8::new(IrqPhase::Idle as u8),
            waker: AtomicWaker::new(),
        }
    }

    pub(crate) fn load_phase(&self) -> IrqPhase {
        IrqPhase::from_u8(self.phase.load(Ordering::Acquire))
    }

    pub(crate) fn set_phase(&self, phase: IrqPhase) {
        self.phase.store(phase as u8, Ordering::Release);
    }

    pub(crate) fn transition(&self, phase: IrqPhase) {
        self.set_phase(phase);
        self.waker.wake();
    }

    async fn wait_for_phase(
        &self,
        timeout: Duration,
        timeout_err: DshotError,
        classify: impl Fn(IrqPhase) -> PhaseDisposition,
    ) -> Result<(), DshotError> {
        with_timeout(
            timeout,
            poll_fn(|cx| {
                self.waker.register(cx.waker());
                match classify(self.load_phase()) {
                    PhaseDisposition::Pending => core::task::Poll::Pending,
                    PhaseDisposition::Done => core::task::Poll::Ready(Ok(())),
                    PhaseDisposition::Error(e) => core::task::Poll::Ready(Err(e)),
                }
            }),
        )
        .await
        .map_err(|_| timeout_err)?
    }

    pub(crate) async fn wait_done(&self, timeout: Duration) -> Result<(), DshotError> {
        self.wait_for_phase(timeout, DshotError::TxTimeout, |phase| match phase {
            IrqPhase::TxActive | IrqPhase::RxActive => PhaseDisposition::Pending,
            IrqPhase::Done => PhaseDisposition::Done,
            IrqPhase::TxError => PhaseDisposition::Error(DshotError::TxDmaError),
            IrqPhase::RxError => PhaseDisposition::Error(DshotError::RxDmaError),
            IrqPhase::Idle => PhaseDisposition::Error(DshotError::TxTimeout),
        })
        .await
    }
}
