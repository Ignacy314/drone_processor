#![allow(non_snake_case)]
use clap::{Parser, Subcommand};
use std::{
    fs::File,
    io::BufWriter,
    io::Write,
    path::{Path, PathBuf},
};

use ekf::{Ekf, Sensor};
use flexi_logger::{Logger, with_thread};
use geoconv::{CoordinateSystem, Degrees, Enu, Lle, Meters, Wgs84};
use regex::Regex;
use serde::Deserialize;

mod ekf;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
#[command(propagate_version = true)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[allow(clippy::enum_variant_names)]
#[derive(Subcommand)]
enum Commands {
    LocationSim(LocationSimArgs),
}

#[derive(clap::Args)]
struct LocationSimArgs {
    #[arg(long)]
    input_dir: String,
    #[arg(long)]
    modules_csv: String,
    #[arg(long)]
    output_csv: String,
    #[arg(long)]
    max_dist: Option<f64>,
}

#[allow(unused)]
#[derive(Deserialize)]
struct ModuleRecord {
    module: i32,
    lat: f64,
    lon: f64,
}

pub fn simulate<P: AsRef<Path>>(
    input_dir: P,
    modules_csv: P,
    output_csv: P,
    max_dist: Option<f64>,
) {
    let re_csv = Regex::new(r".*\D(\d+)\.csv$").unwrap();

    let mut csvs: Vec<PathBuf> = std::fs::read_dir(input_dir)
        .unwrap()
        .map(|d| d.unwrap().path())
        .collect();
    csvs.sort_unstable_by(|a, b| {
        let a_num: i32 = re_csv.captures(a.to_str().unwrap()).unwrap()[1]
            .parse()
            .unwrap();
        let b_num: i32 = re_csv.captures(b.to_str().unwrap()).unwrap()[1]
            .parse()
            .unwrap();
        a_num.cmp(&b_num)
    });

    let mut modules_csv = csv::Reader::from_path(modules_csv).unwrap();

    let mut modules = Vec::new();
    for module in modules_csv.deserialize() {
        let r: ModuleRecord = module.unwrap();
        modules.push(r);
    }

    assert_eq!(modules.len(), csvs.len());

    let mut readers = Vec::new();
    let mut desers = Vec::new();
    for csv in csvs {
        let reader = csv::Reader::from_path(csv).unwrap();
        readers.push(reader);
    }
    for reader in readers.iter_mut() {
        desers.push(reader.deserialize::<f64>());
    }

    let ref_lle = Lle::<Wgs84>::new(
        Degrees::new(modules[0].lat),
        Degrees::new(modules[0].lon),
        Meters::new(0.0),
    );
    let mut sensors: Vec<Sensor> = modules
        .iter()
        .map(|m| {
            let lle = Lle::<Wgs84>::new(Degrees::new(m.lat), Degrees::new(m.lon), Meters::new(0.0));
            let enu = CoordinateSystem::lle_to_enu(&lle, &ref_lle);
            Sensor { enu, dist: 0.0 }
        })
        .collect();

    let mut results = Vec::new();
    let mut ekf = Ekf::new(0.0, 0.0, max_dist);

    let mut counter = 0;

    loop {
        let mut distances = desers.iter_mut().map(|d| d.next());
        if distances.any(|d| d.is_none()) {
            log::info!("Done: {counter}");
            break;
        }
        let distances = distances.map(|d| d.unwrap().unwrap());

        for (sensor, dist) in sensors.iter_mut().zip(distances) {
            sensor.dist = dist;
        }

        let (x_pred, P_pred) = ekf.predict(0.05);
        ekf.update(x_pred, P_pred, &sensors);

        let enu = Enu {
            east: Meters::new(ekf.x_est[0]),
            north: Meters::new(ekf.x_est[1]),
            up: Meters::new(0.0),
        };

        let lle = CoordinateSystem::enu_to_lle(&ref_lle, &enu);

        results.push((lle.latitude.as_float(), lle.longitude.as_float(), lle.elevation.as_float()));
        counter += 1;
    }

    std::fs::create_dir_all(output_csv.as_ref().parent().unwrap()).unwrap();
    let mut csv = BufWriter::new(File::create(output_csv).unwrap());
    writeln!(csv, "lat,lon,alt").unwrap();
    for r in results {
        writeln!(csv, "{},{},{}", r.0, r.1, r.2).unwrap();
    }
}

fn main() {
    Logger::try_with_env_or_str("info")
        .unwrap()
        .log_to_stderr()
        .format(with_thread)
        .use_utc()
        .start()
        .unwrap();

    let cli = Cli::parse();

    match cli.command {
        Commands::LocationSim(args) => {
            simulate(args.input_dir, args.modules_csv, args.output_csv, args.max_dist);
        }
    }
}

// #[cfg(test)]
// mod test {
//     use circular_buffer::CircularBuffer;
//
//     use crate::Point;
//
//     // #[test]
//     // fn point() {
//     //     let new_point = Point { x: 10.0, y: 10.0, z: 0.0 };
//     //     let prev_point = Point { x: 100.0, y: 100.0, z: 0.0 };
//     //
//     //     let time_diff = 1.0;
//     //
//     //     let max_dist = 30.0 * time_diff;
//     //
//     //     let new_point = if time_diff > 1.0 {
//     //         new_point
//     //     } else {
//     //         let dir = prev_point.diff(&new_point).norm();
//     //         assert_almost_eq!(dir.x, -1.0 / 2f64.sqrt(), 1e-6);
//     //         assert_almost_eq!(dir.y, -1.0 / 2f64.sqrt(), 1e-6);
//     //         assert_eq!(dir.z, 0.0);
//     //         let dist = prev_point.dist(&new_point).min(max_dist);
//     //         assert_eq!(dist, 30.0);
//     //
//     //         prev_point.add(&dir.scale(dist))
//     //     };
//     //
//     //     assert_eq!(new_point.x, 70.0);
//     //     assert_eq!(new_point.y, 70.0);
//     //     assert_eq!(new_point.z, 0.0);
//     // }
//
//     #[test]
//     fn circ_avg() {
//         let mut points: CircularBuffer<20, Point> = CircularBuffer::new();
//         points.push_back(Point::new(1.0, 1.0, 1.0));
//
//         let avg_point = points
//             .iter()
//             .fold(Point { x: 0.0, y: 0.0, z: 0.0 }, |a, b| a.add(b));
//         let n = points.len() as f64;
//         let avg_point = avg_point.scale(1.0 / n);
//
//         assert_eq!(avg_point, Point::new(1.0, 1.0, 1.0));
//
//         points.push_back(Point::new(3.0, 3.0, 3.0));
//         let avg_point = points
//             .iter()
//             .fold(Point { x: 0.0, y: 0.0, z: 0.0 }, |a, b| a.add(b));
//         let n = points.len() as f64;
//         let avg_point = avg_point.scale(1.0 / n);
//
//         assert_eq!(avg_point, Point::new(2.0, 2.0, 2.0));
//
//         points.push_back(Point::new(5.0, 2.0, 2.0));
//         let avg_point = points
//             .iter()
//             .fold(Point { x: 0.0, y: 0.0, z: 0.0 }, |a, b| a.add(b));
//         let n = points.len() as f64;
//         let avg_point = avg_point.scale(1.0 / n);
//
//         assert_eq!(avg_point, Point::new(3.0, 2.0, 2.0));
//     }
// }

// #[cfg(test)]
// mod test {
//     use std::collections::HashMap;
//
//     use eqsolver::multivariable::MultiVarNewtonFD;
//     use geoconv::{CoordinateSystem, Degrees, Enu, Lle, Meters, Wgs84};
//     use nalgebra::{Vector3, vector};
//
//     struct Module {
//         pub n: u8,
//         pub lat: f64,
//         pub lon: f64,
//         pub alt: f64,
//         pub dist: f64,
//     }
//
//     #[test]
//     fn solve() {
//         let modules = [
//             Module {
//                 n: 4,
//                 lat: 52.5767238253396,
//                 lon: 16.768104883333333,
//                 alt: 0.0,
//                 dist: 182.51414314060997,
//             },
//             Module {
//                 n: 5,
//                 lat: 52.57671242178453,
//                 lon: 16.76808677936311,
//                 alt: 0.0,
//                 dist: 181.46700014642875,
//             },
//             Module {
//                 n: 6,
//                 lat: 52.57670083223408,
//                 lon: 16.7680328733773,
//                 alt: 0.0,
//                 dist: 180.84368062453862,
//             },
//             Module {
//                 n: 7,
//                 lat: 52.576835055555556,
//                 lon: 16.768655718723892,
//                 alt: 0.0,
//                 dist: 192.76575350451657,
//             },
//         ];
//
//         let lles: HashMap<u8, Lle<Wgs84>> = HashMap::from_iter(modules.iter().map(|m| {
//             (
//                 m.n,
//                 Lle::<Wgs84>::new(Degrees::new(m.lat), Degrees::new(m.lon), Meters::new(m.alt)),
//             )
//         }));
//
//         let (_refr_n, refr) = lles.iter().next().unwrap();
//
//         let enus: HashMap<u8, Enu> = HashMap::from_iter(
//             lles.iter()
//                 .map(|(n, lle)| (*n, CoordinateSystem::lle_to_enu(refr, lle))),
//         );
//
//         let f = |v: Vector3<f64>| {
//             let enu4 = enus.get(&4).unwrap();
//             let enu6 = enus.get(&6).unwrap();
//             let enu7 = enus.get(&7).unwrap();
//             vector![
//                 (v[0] - enu4.east.as_float()).powi(2)
//                     + (v[1] - enu4.north.as_float()).powi(2)
//                     + (v[2] - enu4.up.as_float()).powi(2)
//                     - modules[0].dist.powi(2),
//                 (v[0] - enu6.east.as_float()).powi(2)
//                     + (v[1] - enu6.north.as_float()).powi(2)
//                     + (v[2] - enu6.up.as_float()).powi(2)
//                     - modules[2].dist.powi(2),
//                 (v[0] - enu7.east.as_float()).powi(2)
//                     + (v[1] - enu7.north.as_float()).powi(2)
//                     + (v[2] - enu7.up.as_float()).powi(2)
//                     - modules[3].dist.powi(2),
//             ]
//         };
//
//         let solution = MultiVarNewtonFD::new(f)
//             .with_tol(1e-12)
//             .with_itermax(500)
//             .solve(vector![100.0, 100.0, 100.0])
//             .unwrap();
//
//         // Want to solve x^2 - y = 1 and xy = 2
//         // let f = |v: Vector2<f64>| vector![v[0].powi(2) - v[1] - 1., v[0] * v[1] - 2.];
//
//         // let solution = MultiVarNewtonFD::new(f).solve(vector![1., 1.]); // Starting guess is (1, 1)
//
//         let solution_lle = CoordinateSystem::enu_to_lle(
//             refr,
//             &Enu {
//                 east: Meters::new(solution.x),
//                 north: Meters::new(solution.y),
//                 up: Meters::new(solution.z),
//             },
//         );
//
//         println!("{solution:?}");
//         println!("{solution_lle:?}");
//     }
// }
