.macro section name
.section .\name
.global \name
\name:
.endm

section gic_base
section uart_base
section gpio_base

.section .text

.macro logarm reg
  stp     x29, x30, [sp, #-16]!
  stp     x0, x1, [sp, #-16]!
  stp     x2, x3, [sp, #-16]!
  stp     x4, x5, [sp, #-16]!
  mrs     x4, nzcv                                // preserve N, Z, C, and V flags in x4 (not disturbed by following uart_puts call)
  adrp    x0, msg_\reg
  add     x0, x0, :lo12:msg_\reg
  bl      uart_puts
  msr     nzcv, x4                                // restore flags from x4 (in case nzcv is flag being logged)
  mrs     x0, \reg                                // read register value into x0
  bl      uart_x0
  bl      uart_newline
  msr     nzcv, x4                                // restore nzcv again (since uart_x0 / uart_newline may have disturbed it)
  ldp     x4, x5, [sp], #16
  ldp     x2, x3, [sp], #16
  ldp     x0, x1, [sp], #16
  ldp     x29, x30, [sp], #16
.endm


.align 2
_start:
  mrs     x0, mpidr_el1                           // x0 = Multiprocessor Affinity Register.
  and     x0, x0, #0x3                            // x0 = core number.
  cbnz    x0, sleep_core                          // Put all cores except core 0 to sleep.

  mov     sp, 0x00800000

  bl      uart_init

  mrs     x0, currentel                           // check if already in EL1t mode?
  cmp     x0, #0x4
  b.eq    el1_entry                               // skip ahead, if already at EL1t, no work to do
  ldr     x0, =0x00308000                         // IRQ, FIQ and exception handler run in EL1h
  msr     sp_el1, x0                              // init their stack
  logarm  SP_EL1


  adr     x0, msg_starting
  bl      uart_puts


  mov     x0, 0xff800000
  str     wzr, [x0]
  mov     w1, 0x80000000
  str     w1, [x0, #8]
  mrs     x0, s3_1_c11_c0_2
  mov     x1, #0x22
  orr     x0, x0, x1
  msr     s3_1_c11_c0_2, x0
  ldr     x0, =54000000
  msr     cntfrq_el0, x0
  msr     cptr_el3, xzr
  mov     x0, 0x00000431
  msr     scr_el3, x0
  mov     sp, 0x00800000
  mov     x0, #0x40
  msr     s3_1_c15_c2_1, x0
  bl      setup_gic
  ldr     x0, =0x30c50830
  msr     sctlr_el2, x0
  mov     x0, #0x3c9
  msr     spsr_el3, x0
  adr     x0, in_el2
  msr     elr_el3, x0
  eret
in_el2:

  mov     sp, 0x00800000

  mrs     x0, cnthctl_el2                         // Initialize Generic Timers
  orr     x0, x0, #0x3                            // Enable EL1 access to timers
  msr     cnthctl_el2, x0
  logarm  CNTHCTL_EL2
  msr     cntvoff_el2, xzr
  logarm  CNTVOFF_EL2
  mrs     x0, midr_el1                            // Initilize MPID/MPIDR registers
  mrs     x1, mpidr_el1
  msr     vpidr_el2, x0
  logarm  VPIDR_EL2
  msr     vmpidr_el2, x1
  logarm  VMPIDR_EL2
  mov     x0, #0x33ff                             // Disable coprocessor traps
  msr     cptr_el2, x0                            // Disable coprocessor traps to EL2
  logarm  CPTR_EL2
  msr     hstr_el2, xzr                           // Disable coprocessor traps to EL2
  logarm  HSTR_EL2
  mov     x0, #0x300000
  msr     cpacr_el1, x0                           // Enable FP/SIMD at EL1
  logarm  CPACR_EL1
  mov     x0, #0x80000000                         // 64bit EL1
  msr     hcr_el2, x0
  logarm  HCR_EL2
                                                  // SCTLR_EL1 initialization
                                                  //
                                                  // setting RES1 bits (29,28,23,22,20,11) to 1
                                                  // and RES0 bits (31,30,27,21,17,13,10,6) +
                                                  // UCI,EE,EOE,WXN,nTWE,nTWI,UCT,DZE,I,UMA,SED,ITD,
                                                  // CP15BEN,SA0,SA,C,A,M to 0
  mov     x0, #0x800
  movk    x0, #0x30d0, lsl #16
  msr     sctlr_el1, x0                           // SCTLR_EL1 = 0x30d00800
  logarm  SCTLR_EL1
  mov     x0, #0x3c4                              // Return to the EL1_SP1 mode from EL2
  msr     spsr_el2, x0                            // EL1_SP0 | D | A | I | F
  logarm  SPSR_EL2
  adr     x0, el1_entry
  msr     elr_el2, x0
  logarm  ELR_EL2

  logarm  CNTFRQ_EL0
  logarm  CNTHCTL_EL2
  logarm  CNTHP_CTL_EL2
  logarm  CNTHP_CVAL_EL2
  logarm  CNTHP_TVAL_EL2
  logarm  CNTKCTL_EL1
  logarm  CNTPCT_EL0
  logarm  CNTP_CTL_EL0
  logarm  CNTP_CVAL_EL0
  logarm  CNTP_TVAL_EL0
  logarm  CNTVCT_EL0
  logarm  CNTVOFF_EL2
  logarm  CNTV_CTL_EL0
  logarm  CNTV_CVAL_EL0
  logarm  CNTV_TVAL_EL0

  eret


el1_entry:

  mov     sp, 0x00800000

  adr     x10, mbreq                              // x10 = memory block pointer for mailbox call.
  mov     w11, 8                                  // Mailbox channel 8.
  orr     w2, w10, w11                            // Encoded request address + channel number.
  mov     x3, #0xb880                             // x3 = lower 16 bits of Mailbox Peripheral Address.
  movk    x3, #0xfe00, lsl #16                    // x3 = 0xfe00b880 (Mailbox Peripheral Address)
  1:                                              // Wait for mailbox FULL flag to be clear.
    ldr     w4, [x3, 0x18]                        // w4 = mailbox status.
    tbnz    w4, #31, 1b                           // If FULL flag set (bit 31), try again...
  str     w2, [x3, 0x20]                          // Write request address / channel number to mailbox write register.
  2:                                              // Wait for mailbox EMPTY flag to be clear.
    ldr     w4, [x3, 0x18]                        // w4 = mailbox status.
    tbnz    w4, #30, 2b                           // If EMPTY flag set (bit 30), try again...
  ldr     w4, [x3]                                // w4 = message request address + channel number.
  cmp     w2, w4                                  // See if the message is for us.
  b.ne    2b                                      // If not, try again.
  ldr     w5, [x10, #112]                         // w5 = allocated framebuffer address
  and     w5, w5, #0x3fffffff                     // Clear upper bits beyond addressable memory
  ldr     w6, [x10, #132]                         // w6 = pitch (bytes per horizontal line of framebuffer)
  mov     w11, 1024 * 4                           // w11 = bytes for pixel data per horizontal line of framebuffer
  sub     w6, w6, w11                             // w6 = number of padding bytes at end of horizontal line
  mov     w7, 768                                 // Number of horizontal lines.
  mov     w8, 0x000000ff                          // w8 = RGB encoding for bright red (bright blue in BGR encoding).

  fill_buffer:                                    // Fill the entire framebuffer with red points (pixels).
    mov     w9, 1024                              // w9 = number of points in horizontal line of framebuffer.
    fill_line:                                    // Fill a single row of the framebuffer with red points (pixels).
      str     w8, [x5], 4                         // Make current point bright red, and update x5 to next point.
      sub     w9, w9, 1                           // Decrease horizontal pixel counter.
      cbnz    w9, fill_line                       // Repeat until line complete.
    add     w5, w5, w6                            // Update x5 to start of next line.
    sub     w7, w7, 1                             // Decrease vertical pixel counter.
    cbnz    w7, fill_buffer                       // Repeat until all framebuffer lines complete.

  adr     x0, msg_init_vectors
  bl      uart_puts
  bl      init_vectors

  adr     x0, msg_init_timer
  bl      uart_puts
  bl      init_timer

  adr     x0, msg_enabling_gic
  bl      uart_puts
  bl      enable_gic

  adr     x0, msg_enabling_irq
  bl      uart_puts
  bl      enable_irq

  adrp    x0, gic_base                            // log GICD_* registers
  bl      display_page_32bit
  adrp    x0, (gic_base+0x1000)                   // log GICC_* registers
  bl      display_page_32bit

  adr     x0, msg_done
  bl      uart_puts
  b       sleep_core


setup_gic:
  ldr     x2, =0xff841000
  add     x1, x2, #0x1000
  mov     w0, #0x1e7
  str     w0, [x1]
  mov     w0, #0xff
  str     w0, [x1, #4]
  add     x2, x2, #0x80
  mov     x0, #0x20
  mov     w1, #0xffffffff
  3:
    subs    x0, x0, #0x4
    str     w1, [x2, x0]
    b.ne    3b
  ret


# ------------------------------------------------------------------------------
# Initialise the Mini UART interface for logging over serial port.
# Note, this is Broadcomm's own UART, not the ARM licenced UART interface.
# ------------------------------------------------------------------------------
.align 2
uart_init:
  adrp    x1, uart_base                           // x1 = uart_base
  ldr     w2, [x1, #0x4]                          // w2 = [AUX_ENABLES] (Auxiliary enables)
  orr     w2, w2, #1
  str     w2, [x1, #0x4]                          //   [AUX_ENABLES] |= 0x00000001 => Enable Mini UART.
  str     wzr, [x1, #0x44]                        //   [AUX_MU_IER] = 0x00000000 => Disable Mini UART interrupts.
  str     wzr, [x1, #0x60]                        //   [AUX_MU_CNTL] = 0x00000000 => Disable Mini UART Tx/Rx
  mov     w2, #0x6                                // w2 = 6
  str     w2, [x1, #0x48]                         //   [AUX_MU_IIR] = 0x00000006 => Mini UART clear Tx, Rx FIFOs
  mov     w3, #0x3                                // w3 = 3
  str     w3, [x1, #0x4c]                         //   [AUX_MU_LCR] = 0x00000003 => Mini UART in 8-bit mode.
  str     wzr, [x1, #0x50]                        //   [AUX_MU_MCR] = 0x00000000 => Set UART1_RTS line high.
  mov     w2, 0x0000021d
  str     w2, [x1, #0x68]                         //   [AUX_MU_BAUD] = 0x0000021d
                                                  //         => baudrate = system_clock_freq/(8*([AUX_MU_BAUD]+1))
                                                  //                       (as close to 115200 as possible)
  adrp    x4, gpio_base                           // x4 = [gpio_base] = 0xfe200000
  ldr     w2, [x4, #0x4]                          // w2 = [GPFSEL1]
  and     w2, w2, #0xfffc0fff                     // Unset bits 12, 13, 14 (FSEL14 => GPIO Pin 14 is an input).
                                                  // Unset bits 15, 16, 17 (FSEL15 => GPIO Pin 15 is an input).
  orr     w2, w2, #0x00002000                     // Set bit 13 (FSEL14 => GPIO Pin 14 takes alternative function 5).
  orr     w2, w2, #0x00010000                     // Set bit 16 (FSEL15 => GPIO Pin 15 takes alternative function 5).
  str     w2, [x4, #0x4]                          //   [GPFSEL1] = updated value => Enable UART 1.
  str     wzr, [x4, #0x94]                        //   [GPPUD] = 0x00000000 => GPIO Pull up/down = OFF
  mov     x5, #0x96                               // Wait 150 instruction cycles (as stipulated by datasheet).
  3:
    subs    x5, x5, #0x1                          // x0 -= 1
    b.ne    3b                                    // Repeat until x0 == 0.
  mov     w2, #0xc000                             // w2 = 2^14 + 2^15
  str     w2, [x4, #0x98]                         //   [GPPUDCLK0] = 0x0000c000 => Control signal to lines 14, 15.
  mov     x0, #0x96                               // Wait 150 instruction cycles (as stipulated by datasheet).
  4:
    subs    x0, x0, #0x1                          // x0 -= 1
    b.ne    4b                                    // Repeat until x0 == 0.
  str     wzr, [x4, #0x98]                        //   [GPPUDCLK0] = 0x00000000 => Remove control signal to lines 14, 15.
  str     w3, [x1, #0x60]                         //   [AUX_MU_CNTL] = 0x00000003 => Enable Mini UART Tx/Rx
  ret


.align 2
sleep_core:
  wfe                                             // Sleep until woken.
  b       sleep_core                              // Go back to sleep.


# Send byte in w0 over uart
.align 2
uart_send:
  adrp    x1, uart_base                           // x1 = uart_base
  1:
    ldr     w2, [x1, #0x54]                       // w2 = [AUX_MU_LSR]
    tbz     x2, #5, 1b                            // Repeat last statement until bit 5 is set.
  strb    w0, [x1, #0x40]                         //   [AUX_MU_IO] = w0
  ret


# ------------------------------------------------------------------------------
# Send a null terminated string over Mini UART.
# ------------------------------------------------------------------------------
#
# On entry:
#   x0 = address of null terminated string
# On exit:
#   x0 = address of null terminator
#   x1 = [uart_base] = 0x3f215000 (rpi3) or 0xfe215000 (rpi4)
#   x2 = 0
#   x3 = [AUX_MU_LSR]
.align 2
uart_puts:
  adrp    x1, uart_base                           // x1 = uart_base
  1:
    ldrb    w2, [x0], #1
    cbz     w2, 3f
    2:
      ldr     w3, [x1, #0x54]                     // w3 = [AUX_MU_LSR]
      tbz     x3, #5, 2b                          // Repeat last statement until bit 5 is set.
    strb    w2, [x1, #0x40]                       //   [AUX_MU_IO] = w2
    b       1b
3:
  ret


.align 2
enable_gic:
  adrp    x1, gic_base                            // x1 = GICD base address

  str     wzr, [x1]                               // [0xff841000]     = [GICD_CTLR]        = 0x00000000                                             => disable GIC distributor
  mov     w3, #0x8
  mov     w4, #0xffffffff
  add     x5, x1, #0x180
  1:
    str     w4, [x5, #0x200]                      // [0xff841380+4*n] = [GICD_ICACTIVERn]  = 0xffffffff for n=0 to 7 (1 bit per interrupt)          => clear all 256 active interrupts
    str     w4, [x5, #0x100]                      // [0xff841280+4*n] = [GICD_ICPENDRn]    = 0xffffffff for n=0 to 7 (1 bit per interrupt)          => clear all 256 pending interrupts
    str     w4, [x5], #0x4                        // [0xff841180+4*n] = [GICD_ICENABLERn]  = 0xffffffff for n=0 to 7 (1 bit per interrupt)          => disable all 256 interrupts
    subs    w3, w3, #1
    b.ne    1b
  mov     w3, #0x40
  mov     w6, #0x01010101
  ldr     w7, =0xa0a0a0a0
  add     x5, x1, #0x400
  2:
    str     w6, [x5, #0x400]                      // [0xff841800+4*n] = [GICD_ITARGETSRn]  = 0x01010101 for n=0 to 63 (0x3f) (8 bits per interrupt) => route all 256 interrupts to core 0
    str     w7, [x5], #0x4                        // [0xff841400+4*n] = [GICD_IPRIORITYRn] = 0xa0a0a0a0 for n=0 to 63 (0x3f) (8 bits per interrupt) => set all 256 interrupts priority to 0xa0
    subs    w3, w3, #1
    b.ne    2b
  mov     w3, #0x10
  mov     w8, #0x55555555
  add     x5, x1, #0xc00
  3:
    str     w8, [x5], #0x4                        // [0xff841c00+4*n] = [GICD_ICFGRn]      = 0x55555555 for n=0 to 15 (0xf) (2 bits per interrupt) => set all 256 interrupts to level-sensitive
    subs    w3, w3, #1
    b.ne    3b
  mov     w4, #0x1
  str     w4, [x1]                                // [0xff841000]     = [GICD_CTLR]        = 0x00000001                                            => forward group 1 interrupts from GIC distributor
  mov     w5, #0xf0
  str     w5, [x1, #0x1004]                       // [0xff842004]     = [GICC_PMR]         = 0x000000f0                                            => priority mask = 0xf0
  mov     w6, #0x261
  str     w6, [x1, #0x1000]                       // [0xff842000]     = [GICC_CTLR]        = 0x00000261                                            => EOImodeNS: 1, IRQBypDisGrp1: 1, FIQBypDisGrp1: 1, EnableGrp1: 1

  mov     w4, #0x40000000
  str     w4, [x1, #0x100]                        // [0xff841100]     = [GICD_ISENABLER0]  = 0x40000000                                            => enable interrupt 30 (0x1e)

# mov     w4, #0x00200000
# str     w4, [x1, #0x104]                        // [0xff841104]     = [GICD_ISENABLER1]  = 0x00200000                                            => enable interrupt 53 (0x35)

# enable all interrupts

# mov     w4, #0xffffffff
# str     w4, [x1, #0x100]                        // [0xff841100]     = [GICD_ISENABLER0]  = 0xffffffff                                            => enable interrupts   0- 31 (0x00-0x1f)
# str     w4, [x1, #0x104]                        // [0xff841104]     = [GICD_ISENABLER1]  = 0xffffffff                                            => enable interrupts  32- 63 (0x20-0x3f)
# str     w4, [x1, #0x108]                        // [0xff841108]     = [GICD_ISENABLER2]  = 0xffffffff                                            => enable interrupts  64- 95 (0x40-0x5f)
# str     w4, [x1, #0x10c]                        // [0xff84110c]     = [GICD_ISENABLER3]  = 0xffffffff                                            => enable interrupts  96-127 (0x60-0x7f)
# str     w4, [x1, #0x110]                        // [0xff841110]     = [GICD_ISENABLER4]  = 0xffffffff                                            => enable interrupts 128-159 (0x80-0x9f)
# str     w4, [x1, #0x114]                        // [0xff841114]     = [GICD_ISENABLER5]  = 0xffffffff                                            => enable interrupts 160-191 (0xa0-0xbf)
# str     w4, [x1, #0x118]                        // [0xff841118]     = [GICD_ISENABLER6]  = 0xffffffff                                            => enable interrupts 192-223 (0xc0-0xdf)
# str     w4, [x1, #0x11c]                        // [0xff84111c]     = [GICD_ISENABLER7]  = 0xffffffff                                            => enable interrupts 224-255 (0xe0-0xff)

  ret


.align 2
enable_irq:
  msr     daifclr, #2
  ret


.align 2
init_vectors:
  adr     x0, vectors                             // load VBAR_EL1 with virtual
  msr     vbar_el1, x0                            // vector table address
  logarm  VBAR_EL1
  ret


# Sets a timer for 200,000 ticks in the future, i.e.
# [mailbox_base + 0x10] = [mailbox_base + 0x4] + 200,000
#
# On exit:
#   x0: mailbox_base
#   x1: new timer value ([mailbox_base + 0x4] + 200000)
#   x2: 1
.align 2
init_timer:
  mrs     x1, cntp_cval_el0
  ldr     w2, =200000                             // TODO: this value should be dependent on clock speed (different for rpi3/rpi4)
  add     x1, x1, x2
  msr     cntp_cval_el0, x1
  logarm  CNTP_CVAL_EL0
  mov     w2, #0x1
  msr     cntp_ctl_el0, x2
  logarm  CNTP_CTL_EL0
  ret


# On exit:
#   <depends on timed_interrupt>
#   x0: mailbox_base
#   x1: 0x2
#   x2: 200000
handle_timer_irq:
  stp     x29, x30, [sp, #-16]!                   // Push frame pointer, procedure link register on stack.
  mov     x29, sp                                 // Update frame pointer to new stack location.
  mrs     x1, cntp_cval_el0
  mov     x2, #0x2000000                          // TODO: this value should be dependent on clock speed (different for rpi3/rpi4)
  add     x1, x1, x2
  msr     cntp_cval_el0, x1
  dsb     sy
  bl      timed_interrupt
  ldp     x29, x30, [sp], #0x10                   // Pop frame pointer, procedure link register off stack.
  ret


timed_interrupt:
  stp     x29, x30, [sp, #-16]!                   // Push frame pointer, procedure link register on stack.
  mov     x29, sp                                 // Update frame pointer to new stack location.
# logarm  CNTP_CVAL_EL0
  mov     x0, x7
  bl      uart_x0
  bl      uart_newline
  ldp     x29, x30, [sp], #0x10                   // Pop frame pointer, procedure link register off stack.
  ret


show_invalid_entry_message:
  stp     x29, x30, [sp, #-16]!                   // Push frame pointer, procedure link register on stack.
  mov     x29, sp                                 // Update frame pointer to new stack location.
  adr     x0, msg_invalid_entry
  bl      uart_puts
  mov     x0, x4
  bl      uart_x0
  bl      uart_newline
  mov     x0, x5
  bl      uart_x0
  bl      uart_newline
  mov     x0, x6
  bl      uart_x0
  bl      uart_newline
  ldp     x29, x30, [sp], #0x10                   // Pop frame pointer, procedure link register off stack.
  ret


handle_irq:
  stp     x29, x30, [sp, #-16]!                   // Push frame pointer, procedure link register on stack.
  mov     x29, sp                                 // Update frame pointer to new stack location.
  adrp    x8, (gic_base+0x1000)
  ldr     w7, [x8, #0xc]                          // w7 = [0xff84200c] = [GICC_IAR]
  str     w7, [x8, #0x10]                         // [0xff842010] = [GICC_EOIR] = [GICC_IAR]
                                                  // Note: Writing to GICC_EOIR before servicing interrupt, which I believe means the
                                                  // interrupt routine will be reentrant at this point. Writing to EOIR after
                                                  // handling timer may be safer.
  dsb     sy                                      // The GIC architecture specification requires that valid EOIR writes are ordered
                                                  // however probably not needed since device memory writes should already be ordered.
  bl      handle_timer_irq
  str     w7, [x8, #0x1000]                       // [0xff843000] = [GICC_DIR]  = [GICC_IAR]
                                                  // Note: Could set GICC_CTLR.EOImodeNS to 0 and not have separate GICC_EOIR and
                                                  // GICC_DIR writes, i.e. just write to GICC_EOIR after servicing interrupt.

  ldp     x29, x30, [sp], #16                     // Pop frame pointer, procedure link register off stack.
  ret


# ------------------------------------------------------------------------------
# Write the full value of x0 as hex string (0x0123456789abcdef) to Mini UART.
# Sends 18 bytes ('0', 'x', <16 byte hex string>). No trailing newline.
# ------------------------------------------------------------------------------
#
# On entry:
#   x0 = value to write as a hex string to Mini UART.
#   x2 = number of bits to print (multiple of 4)
# On exit:
#   x0 preserved
#   x1 = [uart_base] = 0x3f215000 (rpi3) or 0xfe215000 (rpi4)
#   x2 = 0
#   x3 = [AUX_MU_LSR]
uart_x0:
  stp     x29, x30, [sp, #-16]!                   // Push frame pointer, procedure link register on stack.
  mov     x29, sp                                 // Update frame pointer to new stack location.
  mov     x2, #64
  bl      uart_x0_s
  ldp     x29, x30, [sp], #16                     // Pop frame pointer, procedure link register off stack.
  ret


# ------------------------------------------------------------------------------
# Write the lower nibbles of x0 as a hex string to Mini UART, with custom size
# (number of nibbles).
# ------------------------------------------------------------------------------
#
# Includes leading '0x' and no trailing newline.
#
# On entry:
#   x0 = value to write as a hex string to Mini UART.
#   x2 = number of lower bits of x0 to print (multiple of 4)
# On exit:
#   x0 preserved
#   x1 = [uart_base] = 0x3f215000 (rpi3) or 0xfe215000 (rpi4)
#   x2 = 0
#   x3 = [AUX_MU_LSR]
uart_x0_s:
  stp     x29, x30, [sp, #-16]!                   // Push frame pointer, procedure link register on stack.
  mov     x29, sp                                 // Update frame pointer to new stack location.
  stp     x19, x20, [sp, #-16]!                   // Backup x19, x20
  mov     x19, x0                                 // Backup x0 in x19
  sub     sp, sp, #0x20                           // Allocate space on stack for hex string
  mov     w3, #0x7830
  mov     x1, sp
  strh    w3, [x1], #2                            // "0x"
  bl      hex_x0
  strb    wzr, [x1], #1
  mov     x0, sp
  bl      uart_puts
  add     sp, sp, #0x20
  mov     x0, x19                                 // Restore x0
  ldp     x19, x20, [sp], #16                     // Restore x19, x20
  ldp     x29, x30, [sp], #16                     // Pop frame pointer, procedure link register off stack.
  ret


# On entry:
#   x0 = hex value to convert to text
#   x1 = address to write text to (no trailing 0)
#   x2 = number of bits to convert (multiple of 4)
# On exit:
#   x0 = <unchanged>
#   x1 = address of next unused char (x1 += x2/4)
#   x2 = 0
#   x3 = last char in text
hex_x0:
  ror     x0, x0, x2
1:
  ror     x0, x0, #60
  and     w3, w0, #0x0f
  add     w3, w3, 0x30
  cmp     w3, 0x3a
  b.lo    2f
  add     w3, w3, 0x27
2:
  strb    w3, [x1], #1
  subs    w2, w2, #4
  b.ne    1b
  ret


# ------------------------------------------------------------------------------
# Send '\r\n' over Mini UART
# ------------------------------------------------------------------------------
#
# On entry:
#   <nothing>
# On exit:
#   x0: 0x0a
#   x1: 0xfe215000
#   x2: Last read of [AUX_MU_LSR] when waiting for bit 5 to be set
uart_newline:
  stp     x29, x30, [sp, #-16]!                   // Push frame pointer, procedure link register on stack.
  mov     x29, sp                                 // Update frame pointer to new stack location.
  mov     x0, #13
  bl      uart_send
  mov     x0, #10
  bl      uart_send
  ldp     x29, x30, [sp], #0x10                   // Pop frame pointer, procedure link register off stack.
  ret


display_page_32bit:
  stp     x29, x30, [sp, #-16]!                   // Push frame pointer, procedure link register on stack.
  mov     x29, sp                                 // Update frame pointer to new stack location.
  add     x4, x0, #0x1000
  1:
    mov     x2, #36
    bl      uart_x0_s
    mov     x5, x0
    mov     x0, ':'
    bl      uart_send
    mov     x0, ' '
    bl      uart_send
    ldr     w0, [x5], #0x4
    mov     x2, #32
    bl      uart_x0_s
    bl      uart_newline
    mov     x0, x5
    cmp     x0, x4
    b.lt    1b
  ldp     x29, x30, [sp], #0x10                   // Pop frame pointer, procedure link register off stack.
  ret


.macro handle_invalid_entry type
  kernel_entry
  mov     x4, #\type
  mrs     x5, esr_el1
  mrs     x6, elr_el1
  bl      show_invalid_entry_message
  b       sleep_core
.endm


.macro ventry label
  .align 7
  b       \label
.endm


.macro kernel_entry
  sub     sp, sp, #16 * 16
  stp     x0, x1, [sp, #16 * 0]
  stp     x2, x3, [sp, #16 * 1]
  stp     x4, x5, [sp, #16 * 2]
  stp     x6, x7, [sp, #16 * 3]
  stp     x8, x9, [sp, #16 * 4]
  stp     x10, x11, [sp, #16 * 5]
  stp     x12, x13, [sp, #16 * 6]
  stp     x14, x15, [sp, #16 * 7]
  stp     x16, x17, [sp, #16 * 8]
  stp     x18, x19, [sp, #16 * 9]
  stp     x20, x21, [sp, #16 * 10]
  stp     x22, x23, [sp, #16 * 11]
  stp     x24, x25, [sp, #16 * 12]
  stp     x26, x27, [sp, #16 * 13]
  stp     x28, x29, [sp, #16 * 14]
  str     x30, [sp, #16 * 15]
.endm


.macro kernel_exit
  ldp     x0, x1, [sp, #16 * 0]
  ldp     x2, x3, [sp, #16 * 1]
  ldp     x4, x5, [sp, #16 * 2]
  ldp     x6, x7, [sp, #16 * 3]
  ldp     x8, x9, [sp, #16 * 4]
  ldp     x10, x11, [sp, #16 * 5]
  ldp     x12, x13, [sp, #16 * 6]
  ldp     x14, x15, [sp, #16 * 7]
  ldp     x16, x17, [sp, #16 * 8]
  ldp     x18, x19, [sp, #16 * 9]
  ldp     x20, x21, [sp, #16 * 10]
  ldp     x22, x23, [sp, #16 * 11]
  ldp     x24, x25, [sp, #16 * 12]
  ldp     x26, x27, [sp, #16 * 13]
  ldp     x28, x29, [sp, #16 * 14]
  ldr     x30, [sp, #16 * 15]
  add     sp, sp, #16 * 16
  dsb     sy
  isb
  eret
.endm


.align 2
sync_invalid_el1t:
  handle_invalid_entry  0

irq_invalid_el1t:
  kernel_entry
  bl      handle_irq
  kernel_exit

fiq_invalid_el1t:
  handle_invalid_entry  2

error_invalid_el1t:
  handle_invalid_entry  3

sync_invalid_el1h:
  handle_invalid_entry  4

el1_irq:
  handle_invalid_entry  5

fiq_invalid_el1h:
  handle_invalid_entry  6

error_invalid_el1h:
  handle_invalid_entry  7

sync_invalid_el0_64:
  handle_invalid_entry  8

irq_invalid_el0_64:
  handle_invalid_entry  9

fiq_invalid_el0_64:
  handle_invalid_entry  10

error_invalid_el0_64:
  handle_invalid_entry  11

sync_invalid_el0_32:
  handle_invalid_entry  12

irq_invalid_el0_32:
  handle_invalid_entry  13

fiq_invalid_el0_32:
  handle_invalid_entry  14

error_invalid_el0_32:
  handle_invalid_entry  15


# Memory block for mailbox call
.align 4
mbreq:
  .word 140                                       // Buffer size
  .word 0                                         // Request/response code
  .word 0x48003                                   // Tag 0 - Set Screen Size
  .word 8                                         //   value buffer size
  .word 0                                         //   request: should be 0          response: 0x80000000 (success) / 0x80000001 (failure)
  .word 1024                                      //   request: width                response: width
  .word 768                                       //   request: height               response: height
  .word 0x48004                                   // Tag 1 - Set Virtual Screen Size
  .word 8                                         //   value buffer size
  .word 0                                         //   request: should be 0          response: 0x80000000 (success) / 0x80000001 (failure)
  .word 1024                                      //   request: width                response: width
  .word 768                                       //   request: height               response: height
  .word 0x48009                                   // Tag 2 - Set Virtual Offset
  .word 8                                         //   value buffer size
  .word 0                                         //   request: should be 0          response: 0x80000000 (success) / 0x80000001 (failure)
  .word 0                                         //   request: x offset             response: x offset
  .word 0                                         //   request: y offset             response: y offset
  .word 0x48005                                   // Tag 3 - Set Colour Depth
  .word 4                                         //   value buffer size
  .word 0                                         //   request: should be 0          response: 0x80000000 (success) / 0x80000001 (failure)
                                                  //                   32 bits per pixel => 8 red, 8 green, 8 blue, 8 alpha
                                                  //                   See https://en.wikipedia.org/wiki/RGBA_color_space
  .word 32                                        //   request: bits per pixel       response: bits per pixel
  .word 0x48006                                   // Tag 4 - Set Pixel Order (really is "Colour Order", not "Pixel Order")
  .word 4                                         //   value buffer size
  .word 0                                         //   request: should be 0          response: 0x80000000 (success) / 0x80000001 (failure)
  .word 1                                         //   request: 0 => BGR, 1 => RGB   response: 0 => BGR, 1 => RGB
  .word 0x40001                                   // Tag 5 - Get (Allocate) Buffer
  .word 8                                         //   value buffer size (response > request, so use response size)
  .word 0                                         //   request: should be 0          response: 0x80000000 (success) / 0x80000001 (failure)
  .word 4096                                      //   request: alignment in bytes   response: frame buffer base address
  .word 0                                         //   request: padding              response: frame buffer size in bytes
  .word 0x40008                                   // Tag 6 - Get Pitch (bytes per line)
  .word 4                                         //   value buffer size
  .word 0                                         //   request: should be 0          response: 0x80000000 (success) / 0x80000001 (failure)
  .word 0                                         //   request: padding              response: bytes per line
  .word 0                                         // End Tags


/*
 * Exception vectors.
 */
.align 11
vectors:
  ventry  sync_invalid_el1t                       // Synchronous EL1t
  ventry  irq_invalid_el1t                        // IRQ EL1t
  ventry  fiq_invalid_el1t                        // FIQ EL1t
  ventry  error_invalid_el1t                      // Error EL1t

  ventry  sync_invalid_el1h                       // Synchronous EL1h
  ventry  el1_irq                                 // IRQ EL1h
  ventry  fiq_invalid_el1h                        // FIQ EL1h
  ventry  error_invalid_el1h                      // Error EL1h

  ventry  sync_invalid_el0_64                     // Synchronous 64-bit EL0
  ventry  irq_invalid_el0_64                      // IRQ 64-bit EL0
  ventry  fiq_invalid_el0_64                      // FIQ 64-bit EL0
  ventry  error_invalid_el0_64                    // Error 64-bit EL0

  ventry  sync_invalid_el0_32                     // Synchronous 32-bit EL0
  ventry  irq_invalid_el0_32                      // IRQ 32-bit EL0
  ventry  fiq_invalid_el0_32                      // FIQ 32-bit EL0
  ventry  error_invalid_el0_32                    // Error 32-bit EL0


msg_init_vectors:
  .asciz "Initialising vectors...\r\n"
msg_init_timer:
  .asciz "Initialising timer...!\r\n"
msg_enabling_gic:
  .asciz "Enabling GIC...\r\n"
msg_enabling_irq:
  .asciz "Enabling interrupts...\r\n"
msg_done:
  .asciz "Done!\r\n"
msg_invalid_entry:
  .asciz "Invalid Entry!\r\n"
msg_starting:
  .asciz "Starting!\r\n"

.macro msgreg regname
msg_\regname:
.asciz "\regname: "
.endm

msgreg  CNTFRQ_EL0
msgreg  CNTHCTL_EL2
msgreg  CNTHP_CTL_EL2
msgreg  CNTHP_CVAL_EL2
msgreg  CNTHP_TVAL_EL2
msgreg  CNTKCTL_EL1
msgreg  CNTP_CTL_EL0
msgreg  CNTP_CVAL_EL0
msgreg  CNTP_TVAL_EL0
msgreg  CNTPCT_EL0
msgreg  CNTV_CTL_EL0
msgreg  CNTV_CVAL_EL0
msgreg  CNTV_TVAL_EL0
msgreg  CNTVCT_EL0
msgreg  CNTVOFF_EL2
msgreg  CPACR_EL1
msgreg  CPTR_EL2
msgreg  ELR_EL2
msgreg  HCR_EL2
msgreg  HSTR_EL2
msgreg  SCTLR_EL1
msgreg  SP_EL1
msgreg  SPSR_EL2
msgreg  VBAR_EL1
msgreg  VMPIDR_EL2
msgreg  VPIDR_EL2
