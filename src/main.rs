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
        Command::Run { file, ast } => cmd_run(file, ast),
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

fn cmd_run(file: PathBuf, print_ast: bool) {
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
    let mut rt = vosa::runtime::Runtime::new();
    match rt.execute(&mission) {
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
║           V.O.S.A. Language Reference  v0.1             ║
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
          takeoff(<number>m)
          waypoint(lat: <f64>, lon: <f64>, alt: <number>m)
          hover(<number>s)
          camera(action: record | photo | stop, resolution: "4K")
          return_home()
          land()
      }
  }

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
  vosa run   <file.vosa>        Simulate a mission
  vosa run   <file.vosa> --ast  Show parsed AST + simulate
  vosa check <file.vosa>        Validate without running
  vosa docs                     Show this reference
"#;
