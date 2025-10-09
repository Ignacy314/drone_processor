#![allow(non_snake_case)]
use std::{
    collections::HashMap,
    net::TcpListener,
    sync::Arc,
    thread::{sleep, spawn},
    time::{Duration, Instant},
};

use geoconv::{CoordinateSystem, Degrees, Enu, Lle, Meters, Wgs84};
use parking_lot::Mutex;
use tungstenite::{accept, connect};

use crate::ekf::{Ekf, Sensor};

#[derive(Clone, Copy)]
pub struct Module {
    // pub mac: String,
    // pub ip: String,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub drone: bool,
    pub dist: f64,
    pub updated: Instant,
}

pub fn run(ws_in: &str, ws_out: String) {
    // env_logger::builder()
    //     .filter_level(log::LevelFilter::Info)
    //     .target(env_logger::Target::Stdout)
    //     .build();

    // Logger::try_with_env_or_str("info")
    //     .unwrap()
    //     .log_to_stderr()
    //     .format(with_thread)
    //     .use_utc()
    //     .start()
    //     .unwrap();

    let modules: Arc<Mutex<HashMap<String, Module>>> = Arc::new(Mutex::new(HashMap::new()));

    spawn({
        let modules = modules.clone();
        move || {
            let read_period = Duration::from_millis(50);
            let mut ref_lle = None;
            let mut ekf = Ekf::new(0.0, 0.0, None);
            loop {
                // let client = reqwest::blocking::Client::new();
                let (mut socket, _response) = match connect(format!("ws://{ws_out}")) {
                    Ok(c) => c,
                    Err(e) => {
                        log::error!("Website WebSocket connection error: {e}");
                        sleep(Duration::from_millis(1000));
                        continue;
                    }
                };
                log::info!("Website WebSocket connected");

                sleep(Duration::from_secs(1));

                // match socket.send(tungstenite::Message::Text(
                //     format!("detection,{},{},{}", 52.1, 16.7, 21.2).into(),
                // )) {
                //     Ok(_) => {}
                //     Err(err) => {
                //         log::error!("Error sending drone WebSocket message: {err}");
                //         continue;
                //     }
                // }
                // match client
                //     .post("http://10.66.66.1:8080/andros/publish")
                //     .body(format!("detection,{},{},{}", 52.1, 16.7, 21.2))
                //     .send()
                // {
                //     Ok(_) => {}
                //     Err(err) => {
                //         log::warn!("Failed to make POST request: {err}");
                //     }
                // }

                loop {
                    let start = Instant::now();

                    let mut lock = modules.lock();
                    if ref_lle.is_none() {
                        let m = lock.values().next();
                        if let Some(m) = m {
                            ref_lle = Some(Lle::<Wgs84>::new(
                                Degrees::new(m.lat),
                                Degrees::new(m.lon),
                                Meters::new(0.0),
                            ))
                        }
                    }
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
                            let sensors: Vec<Sensor> = modules
                                .values()
                                .map(|m| {
                                    let lle = Lle::<Wgs84>::new(
                                        Degrees::new(m.lat),
                                        Degrees::new(m.lon),
                                        Meters::new(0.0),
                                    );
                                    let enu = CoordinateSystem::lle_to_enu(
                                        &lle,
                                        ref_lle.as_ref().unwrap(),
                                    );
                                    Sensor { enu, dist: m.dist }
                                })
                                .collect();

                            let (x_pred, P_pred) = ekf.predict(0.05);
                            ekf.update(x_pred, P_pred, &sensors);

                            let enu = Enu {
                                east: Meters::new(ekf.x_est[0]),
                                north: Meters::new(ekf.x_est[1]),
                                up: Meters::new(0.0),
                            };

                            let lle = CoordinateSystem::enu_to_lle(ref_lle.as_ref().unwrap(), &enu);

                            let _ = socket.send(tungstenite::Message::Text(
                                format!("{},{}", lle.longitude.as_float(), lle.latitude.as_float())
                                    .into(),
                            ));

                            // match client
                            //     .post("http://10.66.66.1:8080/andros/publish")
                            //     .body(format!(
                            //         "detection,{},{},{}",
                            //         solution_lle.latitude.as_float(),
                            //         solution_lle.longitude.as_float(),
                            //         solution_lle.elevation.as_float()
                            //     ))
                            //     .send()
                            // {
                            //     Ok(_) => {}
                            //     Err(err) => {
                            //         log::error!("Failed to make POST request: {err}");
                            //         break;
                            //     }
                            // }
                        }
                    } else {
                        log::warn!("No detection");
                    }

                    sleep(read_period.saturating_sub(start.elapsed()));
                }
            }
        }
    });

    let server = TcpListener::bind(ws_in).unwrap();
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
                let Ok(msg) = websocket.read() else {
                    continue;
                };
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
