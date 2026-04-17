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
    },
    /// Validate a .vosa file without executing
    Check {
        /// Path to the .vosa mission file
        file: PathBuf,
    },
    /// Print the VOSA language reference
    Docs,
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Command::Run { file, ast, mavlink, ros2 } => cmd_run(file, ast, mavlink, ros2),
        Command::Check { file }   => cmd_check(file),
        Command::Docs             => cmd_docs(),
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

fn cmd_run(file: PathBuf, print_ast: bool, mavlink: Option<String>, ros2: Option<u32>) {
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
    println!("{} All safety constraints passed.", "✔".green().bold());

    // Execute
    let result = if let Some(domain_id) = ros2 {
        println!("{}", "── ROS 2 DOMAIN EXECUTION ──".magenta().bold());
        match vosa::hw_bridge::Ros2Bridge::new(domain_id) {
            Ok(mut bridge) => bridge.execute(&mission),
            Err(e) => Err(e),
        }
    } else if let Some(conn) = mavlink {
        println!("{}", "── MAVLink Execution ──".magenta().bold());
        let mut bridge = vosa::hw_bridge::MavlinkBridge::new(&conn);
        bridge.execute(&mission)
    } else {
        let mut rt = vosa::runtime::Runtime::new();
        rt.execute(&mission)
    };

    match result {
        Ok(report) => {
            println!("\n{}", "── Execution Log ──".cyan().bold());
            for step in &report.steps {
                println!("  {:>3}  {}", step.index.to_string().dimmed(), step.description);
            }
            println!("\n{}", "── Mission Summary ──".cyan().bold());
            println!("  Mission : {}", report.mission_name.bold());
            println!("  Distance: {:.0} m", report.total_distance_m);
            println!("  Max Alt : {:.1} m", report.max_altitude_m);
            println!("\n{}", "Mission complete.".green().bold());
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
                Ok(()) => println!("{} \"{}\" is valid.", "✔".green().bold(), mission.name),
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
  vosa run   <file.vosa>          Simulate a mission
  vosa run   <file.vosa> --ast    Show parsed AST + simulate
  vosa run   <file.vosa> --mavlink tcp:127.0.0.1:5760
  vosa run   <file.vosa> --ros2 0
  vosa check <file.vosa>          Validate without running
  vosa docs                       Show this reference

─── Examples ────────────────────────────────────────────────
  examples/hello_world.vosa       Minimal takeoff and land
  examples/perimeter_scan.vosa    Geofenced perimeter survey
  examples/reactive_mission.vosa  Reactive triggers (battery, wind, obstacle)
  examples/parallel_survey.vosa   Parallel multi-drone blocks
"#;
