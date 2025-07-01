#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, digital::OutputPin};
use panic_probe as _;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionPio0, Pin},
    pac,
    pio::{PIOExt, ShiftDirection},
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
};
use rp_pico as bsp; // 选择开发板

// WS2812 使用的PIO程序，基于800KHz协议
use pio_proc::pio_asm;

const LED_COUNT: usize = 30;

#[entry]
fn main() -> ! {
    info!("WS2812 start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let led: Pin<_, FunctionPio0, _> = pins.gpio2.into_function();
    let led_pin_id = led.id().num;

    let program = pio_asm!(
        ".wrap_target",
        "bitloop:",
        "out x, 1",        // 拉一位到 x
        "jmp !x zero",     // 为 0 跳 zero
        "set pins, 1 [5]", // '1': 高 7周期
        "set pins, 0 [3]", //      低 3周期
        "jmp bitloop",     // ✅ 返回 bitloop，继续下一个bit
        "zero:",
        "set pins, 1 [2]", // '0': 高 3周期
        "set pins, 0 [5]", //      低 7周期
        "jmp bitloop",     // ✅ 同样回到 bitloop
        ".wrap"
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();

    let (mut sm, _, mut tx) = bsp::hal::pio::PIOBuilder::from_installed_program(installed)
        .out_pins(led_pin_id, 1) // 用于out指令
        .set_pins(led_pin_id, 1) // 用于set指令
        .clock_divisor_fixed_point(15, 64) // 125MHz ÷ 12.5 = 10MHz
        .out_shift_direction(ShiftDirection::Right)
        .autopull(true)
        .pull_threshold(24) // 每次写入一个 RGB（24bit）
        .build(sm0);

    sm.set_pindirs([(led_pin_id, bsp::hal::pio::PinDir::Output)]);
    sm.start();

    // // 构造 GRB 数据（全绿）
    // let mut data: [u32; LED_COUNT] = [0; LED_COUNT];
    // for i in 0..LED_COUNT {
    //     let g = 255u8;
    //     let r = 0u8;
    //     let b = 0u8;
    //     data[i] = ((b as u32) << 16) | ((r as u32) << 8) | (g as u32);
    // }

    // // // // 发送数据
    // for color in data {
    //     while tx.is_full() {}
    //     tx.write(color);
    // }

    // // // // WS2812 需要 50us 以上的低电平来“复位”，用延时实现
    // timer.delay_us(80);

    // info!("WS2812 done");

    // loop {
    //     // // // 发送数据
    //     // for color in data {
    //     //     while tx.is_full() {}
    //     //     tx.write(color);
    //     // }

    //     // // // WS2812 需要 50us 以上的低电平来“复位”，用延时实现
    //     // timer.delay_us(80);
    //     cortex_m::asm::wfi();
    // }

    // 彩虹色缓存
    let mut data: [u32; LED_COUNT] = [0; LED_COUNT];

    // 色相角度（Hue）偏移
    let mut hue_offset: f32 = 0.0;

    loop {
        for i in 0..LED_COUNT {
            let hue = (hue_offset + i as f32 * 360.0 / LED_COUNT as f32) % 360.0;
            let (r, g, b) = hsv_to_rgb_f32(hue, 1.0, 0.2);
            data[i] = ((b as u32) << 16) | ((g as u32) << 8) | r as u32; // BGR 顺序
        }

        for color in data {
            while tx.is_full() {}
            tx.write(color);
        }

        // 不调用任何复位延时

        timer.delay_ms(2); // 约60Hz刷新，保证颜色流动平滑且无闪烁

        hue_offset = (hue_offset + 1.0) % 360.0;
    }
}
fn hsv_to_rgb_f32(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let c = v * s;
    let h_prime = h / 60.0;
    let x = c * (1.0 - ((h_prime % 2.0) - 1.0).abs());
    let (r1, g1, b1) = match h_prime as u8 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        5 | _ => (c, 0.0, x),
    };
    let m = v - c;
    (
        ((r1 + m) * 255.0) as u8,
        ((g1 + m) * 255.0) as u8,
        ((b1 + m) * 255.0) as u8,
    )
}
