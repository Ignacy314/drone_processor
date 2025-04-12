use std::{
    collections::HashMap,
    fs::File,
    io::BufWriter,
    io::Write,
    net::TcpListener,
    sync::Arc,
    thread::{sleep, spawn},
    time::{Duration, Instant},
};

use chrono::Utc;
use eqsolver::multivariable::MultiVarNewtonFD;
use flexi_logger::{Logger, with_thread};
use geoconv::{CoordinateSystem, Degrees, Enu, Lle, Meters, Wgs84};
use itertools::Itertools;
use nalgebra::{Vector3, vector};
use parking_lot::Mutex;
use tungstenite::{accept, connect};

#[derive(Clone, Copy)]
struct Module {
    // pub mac: String,
    // pub ip: String,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub drone: bool,
    pub dist: f64,
    pub updated: Instant,
}

#[derive(Clone, Copy, Default, Debug, PartialEq)]
struct Point {
    x: f64,
    y: f64,
    z: f64,
}

fn main() {
    // env_logger::builder()
    //     .filter_level(log::LevelFilter::Info)
    //     .target(env_logger::Target::Stdout)
    //     .build();

    Logger::try_with_env_or_str("info")
        .unwrap()
        .log_to_stderr()
        .format(with_thread)
        .use_utc()
        .start()
        .unwrap();

    let mut csv =
        BufWriter::new(File::create(format!("/home/ignacy/andros/{}.csv", Utc::now())).unwrap());
    writeln!(csv, "time,lat,lon,alt").unwrap();

    let modules: Arc<Mutex<HashMap<String, Module>>> = Arc::new(Mutex::new(HashMap::new()));

    spawn({
        let modules = modules.clone();
        async move || {
            let read_period = Duration::from_millis(50);
            loop {
                let client = reqwest::Client::new();
                let (mut socket, _response) = match connect("ws://127.0.0.1:8080/andros/subscribe")
                {
                    Ok(c) => c,
                    Err(e) => {
                        log::error!("Website WebSocket connection error: {e}");
                        sleep(Duration::from_millis(1000));
                        continue;
                    }
                };
                log::info!("Website WebSocket connected");

                sleep(Duration::from_secs(1));

                match socket.send(tungstenite::Message::Text(
                    format!("detection,{},{},{}", 52.1, 16.7, 21.2).into(),
                )) {
                    Ok(_) => {}
                    Err(err) => {
                        log::error!("Error sending drone WebSocket message: {err}");
                        continue;
                    }
                }
                match client
                    .post("127.0.0.1:8080/andros/publish")
                    .body(format!("detection,{},{},{}", 52.1, 16.7, 21.2))
                    .send()
                    .await
                {
                    Ok(_) => {}
                    Err(err) => {
                        log::warn!("Failed to make POST request: {err}");
                    }
                }

                loop {
                    let start = Instant::now();

                    let mut lock = modules.lock();
                    // retain recently updated modules
                    lock.retain(|_, m| {
                        m.updated.elapsed() < Duration::from_millis(250)
                            && m.lon.is_finite()
                            && m.lat.is_finite()
                    });
                    let modules = lock.clone();
                    drop(lock);

                    let detection = modules.iter().any(|(_, m)| m.drone);

                    if detection {
                        // remove outliers
                        // if modules.len() >= 3 {
                        //     // calculate median distance
                        //     let sorted: Vec<f64> = modules
                        //         .values()
                        //         .map(|m| m.dist)
                        //         .sorted_unstable_by(|a, b| a.total_cmp(b))
                        //         .collect();
                        //
                        //     let n_sorted = sorted.len();
                        //     let median = if n_sorted % 2 == 0 {
                        //         let n_half = n_sorted / 2;
                        //         (sorted[n_half - 1] + sorted[n_half]) / 2.0
                        //     } else {
                        //         sorted[n_sorted / 2]
                        //     };
                        //
                        //     // calcualte average distance
                        //     // let avg = sorted.iter().sum::<f64>() / n_sorted as f64;
                        //
                        //     // retain modules with distance within +/- 25% of median
                        //     modules.retain(|_, m| (median - m.dist).abs() < median / 4.0);
                        // }

                        // proceed with calculating drone position if at least 3 modules retained
                        if modules.len() < 3 {
                            log::warn!("Not enough modules retained to compute solution");
                        } else {
                            // create lle coordinates
                            let lles: HashMap<String, Lle<Wgs84>> =
                                HashMap::from_iter(modules.iter().map(|(mac, m)| {
                                    (
                                        mac.clone(),
                                        Lle::<Wgs84>::new(
                                            Degrees::new(m.lat),
                                            Degrees::new(m.lon),
                                            Meters::new(m.alt),
                                        ),
                                    )
                                }));

                            // set reference for conversion to enu
                            let refr = *lles.iter().next().unwrap().1;

                            // calculate enu coordinates
                            let enus: HashMap<String, Enu> =
                                HashMap::from_iter(lles.into_iter().map(|(n, lle)| {
                                    (n, CoordinateSystem::lle_to_enu(&refr, &lle))
                                }));

                            let mut avg_solution = Point::default();
                            let mut solution_counter = 0u32;

                            // calculate average solution of the distance equation system for all triples
                            for triple in modules.iter().combinations(3) {
                                let f = |v: Vector3<f64>| {
                                    let enus = [
                                        enus.get(triple[0].0).unwrap(),
                                        enus.get(triple[1].0).unwrap(),
                                        enus.get(triple[2].0).unwrap(),
                                    ];
                                    vector![
                                        (v[0] - enus[0].east.as_float()).powi(2)
                                            + (v[1] - enus[0].north.as_float()).powi(2)
                                            + (v[2] - enus[0].up.as_float()).powi(2)
                                            - triple[0].1.dist.powi(2),
                                        (v[0] - enus[1].east.as_float()).powi(2)
                                            + (v[1] - enus[1].north.as_float()).powi(2)
                                            + (v[2] - enus[1].up.as_float()).powi(2)
                                            - triple[1].1.dist.powi(2),
                                        (v[0] - enus[2].east.as_float()).powi(2)
                                            + (v[1] - enus[2].north.as_float()).powi(2)
                                            + (v[2] - enus[2].up.as_float()).powi(2)
                                            - triple[2].1.dist.powi(2),
                                    ]
                                };
                                let Ok(solution) = MultiVarNewtonFD::new(f)
                                    .with_tol(1e-12)
                                    .with_itermax(500)
                                    .solve(vector![100.0, 100.0, 100.0])
                                else {
                                    continue;
                                };

                                avg_solution.x += solution.x;
                                avg_solution.y += solution.y;
                                avg_solution.z += solution.z;

                                solution_counter += 1;
                            }

                            if solution_counter > 0 {
                                avg_solution.x /= solution_counter as f64;
                                avg_solution.y /= solution_counter as f64;
                                avg_solution.z /= solution_counter as f64;

                                let solution_lle = CoordinateSystem::enu_to_lle(
                                    &refr,
                                    &Enu {
                                        east: Meters::new(avg_solution.x),
                                        north: Meters::new(avg_solution.y),
                                        up: Meters::new(avg_solution.z),
                                    },
                                );

                                log::debug!("{avg_solution:?}");
                                log::info!("{solution_lle:?}");

                                writeln!(
                                    csv,
                                    "{},{},{},{}",
                                    Utc::now(),
                                    solution_lle.latitude.as_float(),
                                    solution_lle.longitude.as_float(),
                                    solution_lle.elevation.as_float()
                                )
                                .unwrap();

                                match socket.send(tungstenite::Message::Text(
                                    format!(
                                        "detection,{},{},{}",
                                        solution_lle.latitude.as_float(),
                                        solution_lle.longitude.as_float(),
                                        solution_lle.elevation.as_float()
                                    )
                                    .into(),
                                )) {
                                    Ok(_) => {}
                                    Err(err) => {
                                        log::error!("Error sending drone WebSocket message: {err}");
                                        break;
                                    }
                                }
                            } else {
                                log::warn!("Failed to calculate drone position");
                            }
                        }
                    } else {
                        log::warn!("No detection");
                    }

                    sleep(read_period.saturating_sub(start.elapsed()));
                }
            }
        }
    });

    let server = TcpListener::bind("10.66.66.1:3012").unwrap();
    for stream in server.incoming() {
        let modules = modules.clone();
        spawn(move || {
            // let callback = |req: &Request, mut response: Response| {
            //     println!("Received a new ws handshake");
            //     println!("The request's path is: {}", req.uri().path());
            //     println!("The request's headers are:");
            //     for (header, _value) in req.headers() {
            //         println!("* {header}");
            //     }
            //
            //     // Let's add an additional header to our response to the client.
            //     let headers = response.headers_mut();
            //     headers.append("MyCustomHeader", ":)".parse().unwrap());
            //     headers.append("SOME_TUNGSTENITE_HEADER", "header_value".parse().unwrap());
            //
            //     Ok(response)
            // };
            let mut websocket = accept(stream.unwrap()).unwrap();
            log::info!("WebSocket connection accepted");

            loop {
                let msg = websocket.read().unwrap();
                if msg.is_binary() || msg.is_text() {
                    // log::info!("Message: {msg}");

                    let text = msg.to_text().unwrap();
                    let fields: Vec<&str> = text.split("|").collect();

                    let (mac, ip, lat, lon, drone, dist) = (
                        fields[0],
                        fields[1],
                        fields[2].parse::<f64>().unwrap(),
                        fields[3].parse::<f64>().unwrap(),
                        fields[4].parse::<bool>().unwrap(),
                        fields[5].parse::<f64>().unwrap(),
                    );

                    modules.lock().insert(
                        mac.to_owned(),
                        Module {
                            // mac: mac.to_owned(),
                            // ip: ip.to_owned(),
                            lat,
                            lon,
                            alt: 0.0,
                            drone,
                            dist,
                            updated: Instant::now(),
                        },
                    );

                    log::debug!(
                        "Message {{ mac: {mac}, ip: {ip}, lat: {lat}, lon: {lon}, drone: {drone}, dist: {dist} }}"
                    )
                }
            }
        });
    }
}

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
