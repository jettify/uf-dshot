use core::cell::Cell;
use core::future::poll_fn;
use core::task::Poll;

use critical_section::{CriticalSection, Mutex};
use embassy_stm32_hal::gpio::Pull;
use embassy_stm32_hal::pac;
use embassy_stm32_hal::timer::Channel;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{with_deadline, Duration, Instant};

use super::dma_stream::PreparedRxDmaConfig;
use super::driver::{DshotError, MAX_PORT_MOTORS};
use super::timer_cfg::PacerTimerConfig;

const DMA_IRQ_SLOTS: usize = 8;

#[derive(Clone, Copy)]
pub(crate) struct BidirPinSet {
    pub(crate) gpio: pac::gpio::Gpio,
    pub(crate) pin_nums: [u8; MAX_PORT_MOTORS],
    pub(crate) count: u8,
    pub(crate) pull: Pull,
}

impl BidirPinSet {
    #[inline(always)]
    pub(crate) fn pin_slice(&self) -> &[u8] {
        &self.pin_nums[..self.count as usize]
    }
}

#[derive(Clone, Copy)]
pub(crate) enum TransferState {
    Idle,
    TxStandard {
        timer: pac::timer::TimGp16,
        channel: Channel,
        deadline: Instant,
    },
    TxBidirRelease {
        timer: pac::timer::TimGp16,
        channel: Channel,
        pins: BidirPinSet,
        deadline: Instant,
    },
    TxBidirCapture {
        timer: pac::timer::TimGp16,
        channel: Channel,
        pins: BidirPinSet,
        tx_deadline: Instant,
        rx_timeout: Duration,
        rx_timer_cfg: PacerTimerConfig,
        rx_dma_cfg: PreparedRxDmaConfig,
        rx_buf_addr: u32,
    },
    RxCapture {
        timer: pac::timer::TimGp16,
        channel: Channel,
        deadline: Instant,
    },
    Done,
    TxError,
    RxError,
    TxTimeout,
    RxTimeout,
}

pub(crate) struct DmaSlot {
    state: Mutex<Cell<TransferState>>,
    released_at: Mutex<Cell<Option<Instant>>>,
    waker: AtomicWaker,
}

impl DmaSlot {
    const fn new() -> Self {
        Self {
            state: Mutex::new(Cell::new(TransferState::Idle)),
            released_at: Mutex::new(Cell::new(None)),
            waker: AtomicWaker::new(),
        }
    }

    #[inline(always)]
    pub(crate) fn state(&self, cs: CriticalSection<'_>) -> TransferState {
        self.state.borrow(cs).get()
    }

    #[inline(always)]
    pub(crate) fn set_state(&self, cs: CriticalSection<'_>, state: TransferState) {
        self.state.borrow(cs).set(state);
    }

    pub(crate) fn begin(&self, cs: CriticalSection<'_>, state: TransferState) {
        self.released_at.borrow(cs).set(None);
        self.set_state(cs, state);
    }

    pub(crate) fn record_release(&self, cs: CriticalSection<'_>) -> Instant {
        let released_at = Instant::now();
        self.released_at.borrow(cs).set(Some(released_at));
        released_at
    }

    pub(crate) fn take_released_at(&self) -> Option<Instant> {
        critical_section::with(|cs| self.released_at.borrow(cs).take())
    }

    #[inline(always)]
    pub(crate) fn transition(&self, cs: CriticalSection<'_>, state: TransferState) {
        self.state.borrow(cs).set(state);
        self.waker.wake();
    }

    pub(crate) fn load_state(&self) -> TransferState {
        critical_section::with(|cs| self.state(cs))
    }

    pub(crate) fn is_idle(&self) -> bool {
        matches!(self.load_state(), TransferState::Idle)
    }

    fn classify_tx_done(state: TransferState) -> Option<Result<(), DshotError>> {
        match state {
            TransferState::TxStandard { .. } | TransferState::TxBidirRelease { .. } => None,
            TransferState::Done => Some(Ok(())),
            TransferState::TxError => Some(Err(DshotError::TxDmaError)),
            TransferState::RxError => Some(Err(DshotError::RxDmaError)),
            TransferState::TxTimeout | TransferState::Idle => Some(Err(DshotError::TxTimeout)),
            TransferState::RxTimeout => Some(Err(DshotError::RxTimeout)),
            TransferState::TxBidirCapture { .. } | TransferState::RxCapture { .. } => {
                Some(Err(DshotError::TxTimeout))
            }
        }
    }

    fn classify_tx_handoff(state: TransferState) -> Option<Result<Option<Instant>, DshotError>> {
        match state {
            TransferState::TxBidirCapture { .. } => None,
            TransferState::RxCapture { deadline, .. } => Some(Ok(Some(deadline))),
            TransferState::Done => Some(Ok(None)),
            TransferState::TxError => Some(Err(DshotError::TxDmaError)),
            TransferState::RxError => Some(Err(DshotError::RxDmaError)),
            TransferState::TxTimeout | TransferState::Idle => Some(Err(DshotError::TxTimeout)),
            TransferState::RxTimeout => Some(Err(DshotError::RxTimeout)),
            TransferState::TxStandard { .. } | TransferState::TxBidirRelease { .. } => {
                Some(Err(DshotError::TxTimeout))
            }
        }
    }

    fn classify_rx_done(state: TransferState) -> Option<Result<(), DshotError>> {
        match state {
            TransferState::RxCapture { .. } => None,
            TransferState::Done => Some(Ok(())),
            TransferState::RxError => Some(Err(DshotError::RxDmaError)),
            TransferState::RxTimeout | TransferState::Idle => Some(Err(DshotError::RxTimeout)),
            TransferState::TxError => Some(Err(DshotError::TxDmaError)),
            TransferState::TxTimeout => Some(Err(DshotError::TxTimeout)),
            TransferState::TxStandard { .. }
            | TransferState::TxBidirRelease { .. }
            | TransferState::TxBidirCapture { .. } => Some(Err(DshotError::RxTimeout)),
        }
    }

    pub(crate) async fn wait_tx_done(&self, deadline: Instant) -> Result<(), DshotError> {
        match with_deadline(
            deadline,
            poll_fn(|cx| {
                self.waker.register(cx.waker());
                match Self::classify_tx_done(self.load_state()) {
                    Some(res) => Poll::Ready(res),
                    None => Poll::Pending,
                }
            }),
        )
        .await
        {
            Ok(res) => res,
            Err(_) => {
                Self::classify_tx_done(self.load_state()).unwrap_or(Err(DshotError::TxTimeout))
            }
        }
    }

    /// Waits for the TX->RX handoff in a bidirectional capture transfer.
    ///
    /// Returns `Ok(Some(rx_deadline))` when RX is active, or `Ok(None)` if RX already completed.
    pub(crate) async fn wait_tx_handoff(
        &self,
        tx_deadline: Instant,
    ) -> Result<Option<Instant>, DshotError> {
        match with_deadline(
            tx_deadline,
            poll_fn(|cx| {
                self.waker.register(cx.waker());
                match Self::classify_tx_handoff(self.load_state()) {
                    Some(res) => Poll::Ready(res),
                    None => Poll::Pending,
                }
            }),
        )
        .await
        {
            Ok(res) => res,
            Err(_) => {
                Self::classify_tx_handoff(self.load_state()).unwrap_or(Err(DshotError::TxTimeout))
            }
        }
    }

    pub(crate) async fn wait_rx_done(&self, rx_deadline: Instant) -> Result<(), DshotError> {
        match with_deadline(
            rx_deadline,
            poll_fn(|cx| {
                self.waker.register(cx.waker());
                match Self::classify_rx_done(self.load_state()) {
                    Some(res) => Poll::Ready(res),
                    None => Poll::Pending,
                }
            }),
        )
        .await
        {
            Ok(res) => res,
            Err(_) => {
                Self::classify_rx_done(self.load_state()).unwrap_or(Err(DshotError::RxTimeout))
            }
        }
    }
}

static DMA_SLOTS: [DmaSlot; DMA_IRQ_SLOTS] = [const { DmaSlot::new() }; DMA_IRQ_SLOTS];

#[inline(always)]
pub(crate) fn dma_slot(slot: usize) -> &'static DmaSlot {
    &DMA_SLOTS[slot]
}
