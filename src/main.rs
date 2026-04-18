use clap::Parser as ClapParser;
use colored::Colorize;
use std::path::PathBuf;

/// V.O.S.A. — Vectorized Operational Safety Autonomy
/// The open-source DSL for autonomous drone missions.
#[derive(ClapParser, Debug)]
#[command(
    name = "vosa",
    version = env!("CARGO_PKG_VERSION"),
    author,
    about = "V.O.S.A. — Vectorized Operational Safety Autonomy DSL runner",
    long_about = None,
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(clap::Subcommand, Debug)]
enum Command {
    /// Parse and simulate a .vosa mission file
    Run {
        /// Path to the .vosa mission file
        file: PathBuf,
        /// Print the parsed AST before running
        #[arg(short, long)]
        ast: bool,
        /// MAVLink connection string (e.g. tcp:127.0.0.1:5760). If provided, runs against hardware instead of simulator.
        #[arg(long, value_name = "CONN_STR")]
        mavlink: Option<String>,
        /// ROS 2 Domain ID. If provided, connects natively to the ROS 2 DDS network instead of simulating.
        #[arg(long, value_name = "DOMAIN_ID")]
        ros2: Option<u32>,
        /// Inject fixed sensor values into the simulator, e.g. --inject battery=18,wind=15,roll_angle=0.5
        /// Injected values stay constant (battery won't drain, wind won't escalate).
        /// Use to test whether your trigger conditions fire under specific scenarios.
        #[arg(long, value_name = "KEY=VAL,...")]
        inject: Option<String>,
    },
    /// Validate a .vosa file without executing
    Check {
        /// Path to the .vosa mission file
        file: PathBuf,
    },
    /// Start VOSA as a safety gateway — listens for waypoint commands from an autonomous explorer
    /// and validates each one before forwarding to the drone via MAVLink.
    ///
    /// Protocol: send one command per line, e.g. "waypoint north=50.0 east=10.0 alt=30.0"
    /// VOSA replies "ok" or "error: <reason>" for each command.
    Serve {
        /// Path to the .vosa mission file (used for safety context — geofence, altitude limits, etc.)
        file: PathBuf,
        /// MAVLink connection string
        #[arg(long, value_name = "CONN_STR")]
        mavlink: String,
        /// TCP port to listen on for explorer connections (default: 7777)
        #[arg(long, default_value = "7777")]
        port: u16,
    },
    /// Print the VOSA language reference
    Docs,
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Command::Run {
            file,
            ast,
            mavlink,
            ros2,
            inject,
        } => cmd_run(file, ast, mavlink, ros2, inject),
        Command::Check { file } => cmd_check(file),
        Command::Serve { file, mavlink, port } => cmd_serve(file, mavlink, port),
        Command::Docs => cmd_docs(),
    }
}

fn read_file(path: &PathBuf) -> String {
    match std::fs::read_to_string(path) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{} {e}", "error:".red().bold());
            std::process::exit(1);
        }
    }
}

fn parse_injection(s: &str) -> std::collections::HashMap<String, f64> {
    s.split(',')
        .filter_map(|pair| {
            let mut parts = pair.splitn(2, '=');
            let key = parts.next()?.trim().to_string();
            let val: f64 = parts.next()?.trim().parse().ok()?;
            Some((key, val))
        })
        .collect()
}

fn cmd_run(
    file: PathBuf,
    print_ast: bool,
    mavlink: Option<String>,
    ros2: Option<u32>,
    inject: Option<String>,
) {
    println!("{}", banner());
    let src = read_file(&file);

    // Parse
    let mission = match vosa::parse(&src) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("{} {e}", "parse error:".red().bold());
            std::process::exit(1);
        }
    };

    if print_ast {
        println!("{}\n{:#?}\n", "── AST ──".cyan().bold(), mission);
    }

    // Safety check
    let sandbox = vosa::safety::SafetySandbox::new();
    if let Err(e) = sandbox.validate(&mission) {
        eprintln!("{} {e}", "safety violation:".yellow().bold());
        std::process::exit(2);
    }
    println!("{}", "All safety constraints passed.".green().bold());

    // Execute
    let result = if let Some(domain_id) = ros2 {
        #[cfg(feature = "ros2")]
        {
            println!("{}", "── ROS 2 DOMAIN EXECUTION ──".magenta().bold());
            match vosa::hw_bridge::Ros2Bridge::new(domain_id) {
                Ok(mut bridge) => bridge.execute(&mission),
                Err(e) => Err(e),
            }
        }
        #[cfg(not(feature = "ros2"))]
        {
            let _ = domain_id;
            eprintln!(
                "ROS 2 support is not compiled in. Rebuild with: cargo build --features ros2"
            );
            std::process::exit(1);
        }
    } else if let Some(conn) = mavlink {
        println!("{}", "── MAVLink Execution ──".magenta().bold());
        let mut bridge = vosa::hw_bridge::MavlinkBridge::new(&conn);
        bridge.execute(&mission)
    } else {
        let injected = inject.as_deref().map(parse_injection).unwrap_or_default();
        if !injected.is_empty() {
            println!("{}", "── Sensor Injection ──".yellow().bold());
            for (k, v) in &injected {
                println!("  {} = {}", k.bold(), v);
            }
        }
        let mut rt = vosa::runtime::Runtime::with_injection(injected);
        rt.execute(&mission)
    };

    match result {
        Ok(report) => {
            println!("\n{}", "── Execution Log ──".cyan().bold());
            for step in &report.steps {
                println!(
                    "  {:>3}  {}",
                    step.index.to_string().dimmed(),
                    step.description
                );
            }
            println!("\n{}", "── Mission Summary ──".cyan().bold());
            println!("  Mission : {}", report.mission_name.bold());
            println!("  Distance: {:.0} m", report.total_distance_m);
            println!("  Max Alt : {:.1} m", report.max_altitude_m);
            println!("\n{}", "Mission complete.".green().bold());
        }
        Err(vosa::error::VosaError::FailsafeTriggered(msg)) => {
            eprintln!("{} {msg}", "failsafe triggered:".yellow().bold());
            eprintln!("Mission aborted safely. Check your battery reserve and mission length.");
            std::process::exit(4);
        }
        Err(e) => {
            eprintln!("{} {e}", "runtime error:".red().bold());
            std::process::exit(3);
        }
    }
}

fn cmd_check(file: PathBuf) {
    let src = read_file(&file);
    match vosa::parse(&src) {
        Ok(mission) => {
            let sandbox = vosa::safety::SafetySandbox::new();
            match sandbox.validate(&mission) {
                Ok(()) => println!(
                    "{}",
                    format!("\"{}\" is valid.", mission.name).green().bold()
                ),
                Err(e) => {
                    eprintln!("{} {e}", "safety violation:".yellow().bold());
                    std::process::exit(2);
                }
            }
        }
        Err(e) => {
            eprintln!("{} {e}", "error:".red().bold());
            std::process::exit(1);
        }
    }
}

fn cmd_serve(file: PathBuf, mavlink_conn: String, port: u16) {
    use std::io::{BufRead, BufReader, Write};
    use std::net::TcpListener;

    println!("{}", banner());
    let src = read_file(&file);

    let mission = match vosa::parse(&src) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("{} {e}", "parse error:".red().bold());
            std::process::exit(1);
        }
    };

    let sandbox = vosa::safety::SafetySandbox::new();
    if let Err(e) = sandbox.validate(&mission) {
        eprintln!("{} {e}", "safety violation:".yellow().bold());
        std::process::exit(2);
    }
    println!("{}", "Safety context loaded.".green().bold());
    println!("MAVLink target: {}", mavlink_conn.bold());

    let addr = format!("0.0.0.0:{port}");
    let listener = TcpListener::bind(&addr).expect("failed to bind TCP port");
    println!("{} {}", "Listening for explorer on".green().bold(), addr.bold());
    println!("Protocol: send 'waypoint north=<m> east=<m> alt=<m>' per line");

    for stream in listener.incoming() {
        let Ok(stream) = stream else { continue };
        let peer = stream.peer_addr().map(|a| a.to_string()).unwrap_or_default();
        println!("[serve] Explorer connected from {peer}");

        let mut reader = BufReader::new(stream.try_clone().unwrap());
        let mut writer = stream;
        let mut line = String::new();

        loop {
            line.clear();
            match reader.read_line(&mut line) {
                Ok(0) | Err(_) => {
                    println!("[serve] Explorer disconnected");
                    break;
                }
                Ok(_) => {}
            }

            let line = line.trim();
            if line.is_empty() {
                continue;
            }

            match parse_serve_command(line) {
                Ok((north, east, alt)) => {
                    let cmd = vosa::parser::ast::Command::WaypointRelative { north, east, alt };
                    match sandbox.validate_command(&cmd, &mission.safety) {
                        Ok(()) => {
                            println!("[serve] ✓ waypoint N+{north}m E+{east}m alt {alt}m — safe, forwarding to PX4 (phase 2)");
                            let _ = writeln!(writer, "ok");
                        }
                        Err(e) => {
                            println!("[serve] ✗ safety rejected: {e}");
                            let _ = writeln!(writer, "error: {e}");
                        }
                    }
                }
                Err(e) => {
                    let _ = writeln!(writer, "error: {e}");
                }
            }
        }
    }
}

fn parse_serve_command(line: &str) -> Result<(f64, f64, f64), String> {
    if !line.starts_with("waypoint") {
        return Err(format!("unknown command: {line}"));
    }
    let mut north = None;
    let mut east = None;
    let mut alt = None;
    for part in line.split_whitespace().skip(1) {
        let mut kv = part.splitn(2, '=');
        let key = kv.next().unwrap_or("");
        let val: f64 = kv
            .next()
            .and_then(|v| v.parse().ok())
            .ok_or_else(|| format!("bad value for '{key}'"))?;
        match key {
            "north" => north = Some(val),
            "east" => east = Some(val),
            "alt" => alt = Some(val),
            _ => return Err(format!("unknown param: {key}")),
        }
    }
    Ok((
        north.ok_or("missing north")?,
        east.ok_or("missing east")?,
        alt.ok_or("missing alt")?,
    ))
}

fn cmd_docs() {
    println!("{}", DOCS);
}

fn banner() -> String {
    format!(
        "{}\n{}\n",
        " V . O . S . A .".cyan().bold(),
        " Vectorized Operational Safety Autonomy".dimmed()
    )
}

const DOCS: &str = r#"
╔══════════════════════════════════════════════════════════╗
║           V.O.S.A. Language Reference  v0.2             ║
╚══════════════════════════════════════════════════════════╝

A VOSA program is a single 'mission' block containing optional
vehicle, safety, and flight sub-blocks, plus a required sequence.

─── Structure ───────────────────────────────────────────────

  mission "name" {
      vehicle: <type>

      safety {
          max_altitude:    <number>m
          min_altitude:    <number>m
          max_speed:       <number>m/s
          battery_reserve: <number>%
          geofence:        circle(center: home, radius: <number>m)
          failsafe:        return_home | land | hover
      }

      flight {
          cruise_altitude: <number>m
          cruise_speed:    <number>m/s
      }

      sequence {
          // Commands
          takeoff(<number>m)
          waypoint(lat: <f64>, lon: <f64>, alt: <number>m)
          hover(<number>s)
          camera(action: record | photo | stop, resolution: "4K")
          return_home()
          land()

          // Control flow
          repeat <n> { ... }
          if battery < <n>% { ... }
          parallel { ... }

          // Reactive triggers
          on battery < <n>% { ... }
          on battery > <n>% { ... }
          on wind    < <n>m/s { ... }
          on wind    > <n>m/s { ... }
          on obstacle_detected { ... }
      }
  }

─── Reactive Triggers ───────────────────────────────────────
  Triggers are registered at the point they appear in the sequence
  and remain active for the rest of the mission. The body fires
  once each time the condition transitions from false → true
  (rising-edge semantics). The trigger resets automatically when
  the condition becomes false again.

  Example:
    on battery < 20% {
      return_home()
      land()
    }

    on wind > 12m/s {
      hover(30s)
    }

    on obstacle_detected {
      hover(5s)
      return_home()
      land()
    }

  Supported conditions:
    battery < | > <percent>%      Current battery level
    wind    < | > <speed>m/s      Current wind speed
    obstacle_detected             Sensor obstacle signal

─── Vehicle Types ───────────────────────────────────────────
  Quadcopter | FixedWing | Hexacopter

─── Units ───────────────────────────────────────────────────
  m       metres
  m/s     metres per second
  s       seconds
  %       percent
  deg     degrees

─── Comments ────────────────────────────────────────────────
  // This is a line comment

─── CLI Usage ───────────────────────────────────────────────
  vosa run   <file.vosa>                      Simulate a mission
  vosa run   <file.vosa> --ast                Show parsed AST + simulate
  vosa run   <file.vosa> --mavlink tcp:127.0.0.1:5760
  vosa run   <file.vosa> --ros2 0
  vosa run   <file.vosa> --inject battery=18,wind=15
  vosa check <file.vosa>                      Validate without running
  vosa docs                                   Show this reference

─── Sensor Injection ────────────────────────────────────────
  Override simulator sensor values to test trigger conditions
  without hardware. Injected values remain constant — battery
  won't drain, wind won't escalate.

    --inject battery=18         Hold battery at 18%
    --inject wind=15            Hold wind at 15 m/s
    --inject obstacle=1         Force obstacle_detected = true
    --inject battery=18,wind=15 Multiple overrides, comma-separated
    --inject roll_angle=0.5     Any declared custom sensor

  Use case: verify that your triggers fire at the right thresholds
  before flying real hardware.

─── Sensor Bindings ─────────────────────────────────────────
  Declare a named sensor from any supported MAVLink field,
  then use it in reactive trigger conditions:

    sensor <name> from <MESSAGE>.<field>

  Example:
    sensor roll_angle  from ATTITUDE.roll
    sensor gps_hdop    from GPS_RAW_INT.eph

    on roll_angle > 0.45 { return_home() }
    on gps_hdop   > 500  { hover(20s)    }

  Sensor bindings are validated at compile time (vosa check).
  On real hardware, values come from live MAVLink telemetry.
  In simulation, custom sensors read as 0.0.

  Supported sources:
    ATTITUDE        roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
    VFR_HUD         airspeed, groundspeed, alt, climb
    WIND_COV        wind_x, wind_y
    GPS_RAW_INT     eph, epv, satellites_visible
    SYS_STATUS      battery_remaining
    DISTANCE_SENSOR current_distance

─── Examples ────────────────────────────────────────────────
  examples/hello_world.vosa       Minimal takeoff and land
  examples/perimeter_scan.vosa    Geofenced perimeter survey
  examples/reactive_mission.vosa  Reactive triggers (battery, wind, obstacle)
  examples/parallel_survey.vosa   Parallel multi-drone blocks
"#;
