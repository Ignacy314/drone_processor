use geoconv::Enu;
use nalgebra::{DMatrix, DVector, Matrix4, Vector4};

const INIT_POS_STDDEV: f64 = 800.0;
const INIT_VEL_STDDEV: f64 = 15.0;
const PROCESS_NOISE_STDDEV: f64 = 5.0;
const MEASUREMENT_STDDEV: f64 = 50.0;

#[derive(Clone, Copy, Debug)]
pub struct Sensor {
    pub enu: Enu,
    pub dist: f64,
}

pub struct Ekf {
    pub x_est: Vector4<f64>,
    pub P_est: Matrix4<f64>,
    pub F: Box<dyn Fn(f64) -> Matrix4<f64>>,
    pub Q: Box<dyn Fn(f64) -> Matrix4<f64>>,
    pub max_dist: Option<f64>,
}

impl Ekf {
    pub fn new(x: f64, y: f64, max_dist: Option<f64>) -> Self {
        let x_est = Vector4::new(x, y, 0.0, 0.0);
        let P_est = Matrix4::from_diagonal(&Vector4::new(
            INIT_POS_STDDEV.powi(2),
            INIT_POS_STDDEV.powi(2),
            INIT_VEL_STDDEV.powi(2),
            INIT_VEL_STDDEV.powi(2),
        ));
        // let F = Matrix4::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        let F = Box::new(|dt: f64| {
            Matrix4::new(
                1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            )
        });
        let Q = Box::new(|dt: f64| {
            let q_pos = (PROCESS_NOISE_STDDEV * dt * dt / 2.0).powi(2);
            let q_vel = (PROCESS_NOISE_STDDEV * dt).powi(2);
            Matrix4::from_diagonal(&Vector4::new(q_pos, q_pos, q_vel, q_vel))
        });

        Self { x_est, P_est, F, Q, max_dist }
    }

    pub fn predict(&self, dt: f64) -> (Vector4<f64>, Matrix4<f64>) {
        let F = (self.F)(dt);
        let Q = (self.Q)(dt);
        (F * self.x_est, F * self.P_est * F.transpose() + Q)
    }

    pub fn update(&mut self, x_pred: Vector4<f64>, P_pred: Matrix4<f64>, sensors: &[Sensor]) {
        let filtered_sensors: Vec<Sensor> = sensors
            .iter()
            .filter(|s| {
                s.dist > 0.0 && (self.max_dist.is_none() || s.dist <= self.max_dist.unwrap())
            })
            .copied()
            .collect();
        let n_sensors = filtered_sensors.len();
        if n_sensors < 3 {
            self.x_est = x_pred;
            self.P_est = P_pred;
            return;
        }

        let z = DVector::from_iterator(n_sensors, filtered_sensors.iter().map(|s| s.dist));
        let mut h_x_pred = DVector::zeros(n_sensors);
        let mut H = DMatrix::zeros(n_sensors, 4);
        let R = DMatrix::from_diagonal_element(n_sensors, n_sensors, MEASUREMENT_STDDEV.powi(2));
        let (px, py) = (x_pred[0], x_pred[1]);

        for (i, sensor) in filtered_sensors.iter().enumerate() {
            let (sx, sy) = (-sensor.enu.east.as_float(), -sensor.enu.north.as_float());
            let dist_pred = ((px - sx).powi(2) + (py - sy).powi(2)).sqrt().max(1e-6);
            h_x_pred[i] = dist_pred;
            H[(i, 0)] = (px - sx) / dist_pred;
            H[(i, 1)] = (py - sy) / dist_pred;
        }

        let H_t = H.transpose();
        let S = &H * P_pred * &H_t + R;
        if let Some(S_inv) = S.try_inverse() {
            let K = P_pred * H_t * S_inv;
            let y = z - h_x_pred;
            self.x_est = x_pred + &K * y;
            self.P_est = (Matrix4::identity() - K * H) * P_pred;
        } else {
            self.x_est = x_pred;
            self.P_est = P_pred;
        }
    }
}
