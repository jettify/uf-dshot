use core::marker::PhantomData;
use core::sync::atomic::{fence, Ordering};

use embassy_stm32_hal::dma::Request;
use embassy_stm32_hal::pac;

use super::driver::RawDmaChannel;

#[derive(Clone, Copy)]
pub(crate) struct PreparedRxDmaConfig {
    pub(crate) request: Request,
    pub(crate) peri_addr: u32,
    pub(crate) len: usize,
}

pub(crate) struct DmaStream<D>(PhantomData<D>);

impl<D: RawDmaChannel> DmaStream<D> {
    pub(crate) fn clear_flags() {
        let regs = D::regs();
        let stream = D::stream_num();
        let idx = stream / 4;
        let bit = stream % 4;
        regs.ifcr(idx).write(|w| {
            w.set_feif(bit, true);
            w.set_dmeif(bit, true);
            w.set_teif(bit, true);
            w.set_htif(bit, true);
            w.set_tcif(bit, true);
        });
    }

    pub(crate) fn disable() {
        let regs = D::regs();
        let st = regs.st(D::stream_num());
        st.cr().modify(|w| {
            w.set_en(false);
            w.set_tcie(false);
            w.set_teie(false);
        });
        while st.cr().read().en() {}
        fence(Ordering::Acquire);
    }

    /// Requests stream disable without waiting for EN to clear.
    ///
    /// Intended for interrupt context where bounded latency matters.
    #[inline(always)]
    pub(crate) fn disable_no_wait() {
        let regs = D::regs();
        let st = regs.st(D::stream_num());
        st.cr().modify(|w| {
            w.set_en(false);
            w.set_tcie(false);
            w.set_teie(false);
        });
    }

    #[inline(always)]
    pub(crate) fn is_enabled() -> bool {
        let regs = D::regs();
        regs.st(D::stream_num()).cr().read().en()
    }

    /// Configures and enables the DMA stream.
    ///
    /// # Safety
    /// `mem_addr` and `peri_addr` must be valid for `len` transfers with the selected `dir`/`size`.
    unsafe fn configure_and_start(
        request: Request,
        mem_addr: u32,
        peri_addr: u32,
        len: usize,
        dir: pac::dma::vals::Dir,
        size: pac::dma::vals::Size,
        reset: bool,
    ) {
        let regs = D::regs();
        let st = regs.st(D::stream_num());

        if reset {
            Self::disable();
        }
        // STM32 DMA v2 requires all stream event flags to be cleared before setting EN = 1.
        Self::clear_flags();

        st.par().write_value(peri_addr);
        st.m0ar().write_value(mem_addr);
        st.ndtr().write_value(pac::dma::regs::Ndtr(len as _));
        st.fcr()
            .write(|w| w.set_dmdis(pac::dma::vals::Dmdis::ENABLED));
        fence(Ordering::Release);
        st.cr().write(|w| {
            w.set_dir(dir);
            w.set_msize(size);
            w.set_psize(size);
            w.set_pl(pac::dma::vals::Pl::VERY_HIGH);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_teie(true);
            w.set_tcie(true);
            w.set_htie(false);
            w.set_circ(false);
            w.set_chsel(request);
            w.set_pburst(pac::dma::vals::Burst::SINGLE);
            w.set_mburst(pac::dma::vals::Burst::SINGLE);
            w.set_pfctrl(pac::dma::vals::Pfctrl::DMA);
            w.set_en(true);
        });
    }

    /// Starts a DMA write from memory to peripheral.
    ///
    /// # Safety
    /// `mem_addr` and `peri_addr` must remain valid for `len` 32-bit transfers.
    pub(crate) unsafe fn start_write(
        request: Request,
        mem_addr: *const u32,
        peri_addr: *mut u32,
        len: usize,
    ) {
        Self::configure_and_start(
            request,
            mem_addr as u32,
            peri_addr as u32,
            len,
            pac::dma::vals::Dir::MEMORY_TO_PERIPHERAL,
            pac::dma::vals::Size::BITS32,
            true,
        );
    }

    /// Starts a prepared DMA read without resetting an already configured stream/session.
    ///
    /// # Safety
    /// `mem_addr` and `cfg.peri_addr` must remain valid for `cfg.len` 16-bit transfers.
    pub(crate) unsafe fn start_prepared_read_no_reset(cfg: PreparedRxDmaConfig, mem_addr: u32) {
        Self::configure_and_start(
            cfg.request,
            mem_addr,
            cfg.peri_addr,
            cfg.len,
            pac::dma::vals::Dir::PERIPHERAL_TO_MEMORY,
            pac::dma::vals::Size::BITS16,
            false,
        );
    }
}

#[inline(always)]
pub(crate) fn check_and_clear_dma_irq_flags<D: RawDmaChannel>() -> Option<bool> {
    let regs = D::regs();
    let isr = regs.isr(D::stream_num() / 4).read();
    let bit = D::stream_num() % 4;

    if isr.teif(bit) {
        DmaStream::<D>::clear_flags();
        return Some(false);
    }
    if isr.tcif(bit) {
        regs.ifcr(D::stream_num() / 4)
            .write(|w| w.set_tcif(bit, true));
        return Some(true);
    }
    None
}
