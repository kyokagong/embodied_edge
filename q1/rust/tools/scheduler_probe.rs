//! Read-only Linux timer probe: no ROS, model, IPC or robot commands.
use std::{
    env, thread,
    time::{Duration, Instant},
};
#[repr(C)]
struct SchedParam {
    priority: i32,
}
unsafe extern "C" {
    fn sched_setscheduler(pid: i32, policy: i32, param: *const SchedParam) -> i32;
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.iter().any(|s| s == "--help") {
        println!("scheduler-probe [seconds:1..10] [fifo_priority:0..20]; no robot interfaces");
        return Ok(());
    }
    let seconds: usize = args.first().map_or(Ok(10), |s| s.parse())?;
    let priority: i32 = args.get(1).map_or(Ok(0), |s| s.parse())?;
    if !(1..=10).contains(&seconds) || !(0..=20).contains(&priority) {
        return Err("Out of bounded probe range".into());
    }
    let ticks = seconds * 500;
    let mut gaps = vec![1.0f64; ticks];
    if priority > 0 {
        // Linux SCHED_FIFO=1; pid=0 changes only this single-threaded probe.
        if unsafe { sched_setscheduler(0, 1, &SchedParam { priority }) } != 0 {
            return Err(std::io::Error::last_os_error().into());
        }
    }
    let began = Instant::now();
    let mut deadline = began;
    let mut previous = began;
    let mut over5 = 0;
    for gap in &mut gaps {
        deadline += Duration::from_millis(2);
        thread::sleep(deadline.saturating_duration_since(Instant::now()));
        let now = Instant::now();
        *gap = now.duration_since(previous).as_secs_f64() * 1000.;
        if *gap > 5. {
            over5 += 1;
        }
        if now > deadline + Duration::from_millis(5) {
            deadline = now;
        }
        previous = now;
    }
    let elapsed = began.elapsed().as_secs_f64();
    gaps.sort_by(f64::total_cmp);
    println!("{{\"hardware_command_sent\":false,\"ros_node_created\":false,\"seconds\":{seconds},\"samples\":{ticks},\"fifo_priority\":{priority},\"observed_hz\":{},\"interval_ms\":{{\"p50\":{},\"p99\":{},\"max\":{}}},\"intervals_over_5ms\":{over5}}}",ticks as f64/elapsed,gaps[ticks/2],gaps[ticks*99/100],gaps[ticks-1]);
    Ok(())
}
