#![feature(alloc_error_handler)]
#![feature(fn_align)]
#![no_std]
#![no_main]
#![allow(static_mut_refs)]

extern crate alloc;
#[macro_use]
extern crate log;
#[macro_use]
mod macros;

mod cfg;
mod devicetree;
mod fail;
mod firmware;
mod platform;
mod riscv;
mod sbi;

use core::arch::{asm, naked_asm};

use crate::platform::PLATFORM;
use crate::riscv::csr::menvcfg;
use crate::riscv::current_hartid;
use crate::sbi::features::hart_mhpm_mask;
use crate::sbi::features::{
    Extension, PrivilegedVersion, hart_extension_probe, hart_features_detection,
    hart_privileged_check, hart_privileged_version,
};
use crate::sbi::hart_context::NextStage;
use crate::sbi::heap::sbi_heap_init;
use crate::sbi::hsm::local_remote_hsm;
use crate::sbi::ipi;
use crate::sbi::trap;
use crate::sbi::trap_stack;

pub const R_RISCV_RELATIVE: usize = 3;

#[unsafe(no_mangle)]
extern "C" fn rust_main(_hart_id: usize, opaque: usize, nonstandard_a2: usize) {
    // Track whether SBI is initialized and ready.

    // Get init hart
    let init_hart_info = firmware::get_work_hart(opaque, nonstandard_a2, false);

    // init hart task entry.
    if init_hart_info.is_boot_hart {
        // Initialize the sbi heap
        sbi_heap_init();

        // parse the device tree
        let fdt_address = init_hart_info.fdt_address;

        unsafe {
            PLATFORM.init(fdt_address);
            PLATFORM.print_board_info();
        }

        firmware::set_pmp(unsafe { PLATFORM.info.memory_range.as_ref().unwrap() });
        firmware::log_pmp_cfg(unsafe { PLATFORM.info.memory_range.as_ref().unwrap() });

        // Log boot hart ID and PMP information
        let hart_id = current_hartid();
        info!("{:<30}: {}", "Boot HART ID", hart_id);

        // Detection Hart Features
        hart_features_detection();
        // Other harts task entry.
        trap_stack::prepare_for_trap();
        let priv_version = hart_privileged_version(hart_id);
        let mhpm_mask = hart_mhpm_mask(hart_id);
        info!(
            "{:<30}: {:?}",
            "Boot HART Privileged Version:", priv_version
        );
        info!("{:<30}: {:#08x}", "Boot HART MHPM Mask:", mhpm_mask);
    } else if current_hartid() == 0 {
        trap_stack::prepare_for_trap();

        while !unsafe { PLATFORM.ready() } {
            core::hint::spin_loop()
        }
        firmware::set_pmp(unsafe { PLATFORM.info.memory_range.as_ref().unwrap() });
        // Detection Priv Version
        hart_features_detection();

        #[unsafe(naked)]
        // #[link_section = ".text.hot"] // 将这个小而关键的函数放入热点段是一个好习惯
        unsafe extern "C" fn jump_to_payload(entry: usize, hart_id: usize, opaque: usize) -> ! {
            // 根据 RISC-V 64位 psABI 调用约定:
            // - 第一个参数 `entry`   位于寄存器 a0
            // - 第二个参数 `hart_id` 位于寄存器 a1
            // - 第三个参数 `opaque`  位于寄存器 a2

            // `embassy_app` 的 _start 函数期望:
            // - a0 (argc) = hart_id
            // - a1 (argv) = opaque

            // 我们必须小心地按正确顺序操作寄存器，因为 a0 中有我们需要的入口地址
            unsafe {
                naked_asm!(
                    "csrw mepc, a0", // 1. 将入口地址 (来自 a0) 设置到 mepc，这是第一步，防止 a0 被覆盖
                    "mv   a0, a1",   // 2. 准备 payload 的第一个参数：将 a0 设置为 hart_id (来自 a1)
                    "mv   a1, a2", // 3. prepare payload 的第二个参数：将 a1 设置为 opaque (来自 a2)
                    "mret",        // 4. 执行跳转
                                   // 无需 options(noreturn)，因为 naked_asm! 和 `-> !` 签名已经处理了这一点
                )
            }
        }

        let hart0_dest = 0xc000_0000;

        info!(
            "Hart 0: Redirecting to 0x{:0>16x} in Machine mode.",
            hart0_dest
        );

        unsafe {
            // 设置下一次 mret 后的权限模式为 Machine
            use ::riscv::register::mstatus;
            mstatus::set_mpp(mstatus::MPP::Machine);

            // 调用我们定义好的裸函数来执行跳转。
            // 编译器会根据函数签名和调用约定，自动将参数放入 a0, a1, a2。
            jump_to_payload(
                hart0_dest, // -> a0
                current_hartid(),                  // -> a1
                0x0,                               // -> a2
            );
        }
    } else {
        // Detection Hart feature
        hart_features_detection();
        // Other harts task entry.
        trap_stack::prepare_for_trap();

        // Wait for boot hart to complete SBI initialization.
        while !unsafe { PLATFORM.ready() } {
            core::hint::spin_loop()
        }

        firmware::set_pmp(unsafe { PLATFORM.info.memory_range.as_ref().unwrap() });
    }

    // Get boot information and prepare for kernel entry.
    let boot_info = firmware::get_boot_info(nonstandard_a2);
    let (mpp, next_addr) = (boot_info.mpp, boot_info.next_address);

    // Check hart privileded.
    hart_privileged_check(mpp);

    let boot_hart_info = firmware::get_work_hart(opaque, nonstandard_a2, true);

    // boot hart task entry.
    if boot_hart_info.is_boot_hart {
        unsafe {
            PLATFORM.sbi_cpu_init_with_feature();
        }
        let fdt_address = boot_hart_info.fdt_address;
        let fdt_address = firmware::patch_device_tree(fdt_address);

        // Start kernel.
        local_remote_hsm().start(NextStage {
            start_addr: next_addr,
            next_mode: mpp,
            opaque: fdt_address,
        });

        info!(
            "Redirecting hart {} to {:#016x} in {:?} mode.",
            current_hartid(),
            next_addr,
            mpp
        );
    }

    // Clear all pending IPIs.
    ipi::clear_all();

    // Configure CSRs
    unsafe {
        // Delegate all interrupts and exceptions to supervisor mode.
        asm!("csrw mideleg,    {}", in(reg) !0);
        asm!("csrw medeleg,    {}", in(reg) !0);
        asm!("csrw mcounteren, {}", in(reg) !0);
        asm!("csrw scounteren, {}", in(reg) !0);
        use ::riscv::register::{medeleg, mtvec};
        // Keep supervisor environment calls and illegal instructions in M-mode.
        medeleg::clear_supervisor_env_call();
        medeleg::clear_load_misaligned();
        medeleg::clear_store_misaligned();
        medeleg::clear_illegal_instruction();

        let hart_priv_version = hart_privileged_version(current_hartid());
        if hart_priv_version >= PrivilegedVersion::Version1_11 {
            asm!("csrw mcountinhibit, {}", in(reg) !0b10);
        }
        if hart_priv_version >= PrivilegedVersion::Version1_12 {
            // Configure environment features based on available extensions.
            if hart_extension_probe(current_hartid(), Extension::Sstc) {
                menvcfg::set_bits(
                    menvcfg::STCE | menvcfg::CBIE_INVALIDATE | menvcfg::CBCFE | menvcfg::CBZE,
                );
            } else {
                menvcfg::set_bits(menvcfg::CBIE_INVALIDATE | menvcfg::CBCFE | menvcfg::CBZE);
            }
        }
        // Set up trap handling.
        mtvec::write(fast_trap::trap_entry as _, mtvec::TrapMode::Direct);
    }
}

#[unsafe(naked)]
#[unsafe(link_section = ".text.entry")]
#[unsafe(export_name = "_start")]
unsafe extern "C" fn start() -> ! {
    naked_asm!(
        ".option arch, +a",
        // 1. Turn off interrupt.
        "
        csrw    mie, zero",
        // 2. Initialize programming language runtime.
        // only clear bss if hartid matches preferred boot hart id.
        // Race
        "
            lla      t0, 6f
            li       t1, 1
            amoadd.w t0, t1, 0(t0)
            bnez     t0, 4f
            call     {relocation_update}",
        // 3. Boot hart clear bss segment.
        "1:
            lla     t0, sbi_bss_start
            lla     t1, sbi_bss_end",
        "2:
            bgeu    t0, t1, 3f
            sd      zero, 0(t0)
            addi    t0, t0, 8
            j       2b",
        // 3.1 Boot hart set bss ready signal.
        "3:
            lla     t0, 7f
            li      t1, 1
            amoadd.w t0, t1, 0(t0)
            j       5f",
        // 3.2 Other harts are waiting for bss ready signal.
        "4:
            lla     t0, 7f
            lw      t0, 0(t0)
            beqz    t0, 4b",
        // 4. Prepare stack for each hart.
        "5:
            call    {locate_stack}
            call    {main}
            csrw    mscratch, sp
            j       {hart_boot}
            .balign  4",
        "6:", // boot hart race signal.
        "  .word    0",
        "7:", // bss ready signal.
        "  .word    0",
        relocation_update = sym relocation_update,
        locate_stack = sym trap_stack::locate,
        main         = sym rust_main,
        hart_boot    = sym trap::boot::boot,
    )
}

// Handle relocations for position-independent code
#[unsafe(naked)]
unsafe extern "C" fn relocation_update() {
    naked_asm!(
        // Get load offset.
        "   li t0, {START_ADDRESS}",
        "   lla t1, sbi_start",
        "   sub t2, t1, t0",

        // Foreach rela.dyn and update relocation.
        "   lla t0, __rel_dyn_start",
        "   lla t1, __rel_dyn_end",
        "   li  t3, {R_RISCV_RELATIVE}",
        "1:",
        "   ld  t4, 8(t0)",
        "   bne t4, t3, 2f",
        "   ld t4, 0(t0)", // Get offset
        "   ld t5, 16(t0)", // Get append
        "   add t4, t4, t2", // Add load offset to offset add append
        "   add t5, t5, t2",
        "   sd t5, 0(t4)", // Update address
        "   addi t0, t0, 24", // Get next rela item
        "2:",
        "   blt t0, t1, 1b",
        "   fence.i",

        // Return
        "   ret",
        R_RISCV_RELATIVE = const R_RISCV_RELATIVE,
        START_ADDRESS = const cfg::SBI_LINK_START_ADDRESS,
    )
}
